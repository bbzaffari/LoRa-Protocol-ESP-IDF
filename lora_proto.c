

#include "lora_proto.h"

static TaskHandle_t lora_rx_task_handle = NULL;
// Handler configurável
static lora_packet_handler_t packet_handler = NULL;

void lora_set_packet_handler(lora_packet_handler_t handler) {
    packet_handler = handler;
}
static spi_device_handle_t _spi;
static SemaphoreHandle_t lora_mux;
static uint8_t MY_ID; 
// BaseType_t ACK_PENDENTE;
#define TAG "LoRa_PROTOTIPO"
//______________________________________________________________________________________________
//_________________________ Funcoes de auxiliares ______________________________________________
//______________________________________________________________________________________________

// Escreve em registrador LoRa via SPI
static void lora_write_reg(int reg, int val)
{
    uint8_t out[2] = { (uint8_t)(0x80 | reg), (uint8_t)val };
    spi_transaction_t t = {
        .flags     = 0,
        .length    = 8 * 2,
        .tx_buffer = out,
        .rx_buffer = NULL
    };
    esp_err_t ret = spi_device_transmit(_spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro SPI: %s", esp_err_to_name(ret));
    }
}

// Lê registrador LoRa via SPI
static int lora_read_reg(int reg)
{
    uint8_t out[2] = { (uint8_t)(reg & 0x7F), 0xFF };
    uint8_t in[2]  = { 0 };
    spi_transaction_t t = {
        .flags     = 0,
        .length    = 8 * 2,
        .tx_buffer = out,
        .rx_buffer = in
    };
    esp_err_t ret = spi_device_transmit(_spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Erro SPI: %s", esp_err_to_name(ret));
    }
    return in[1];
}

// Pulso de reset físico no LoRa
static void lora_reset_pulse()
{
    gpio_set_level(CONFIG_RST_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CONFIG_RST_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Leitura de REG_VERSION (0x42) para verificar comunicação SPI
static int read_lora_version()
{
    uint8_t out[2] = { 0x42 & 0x7F, 0x00 };
    uint8_t in[2]  = { 0 };
    spi_transaction_t t = {
        .flags     = 0,
        .length    = 8 * 2,
        .tx_buffer = out,
        .rx_buffer = in
    };
    // controla CS manualmente
    gpio_set_level(CONFIG_CS_GPIO, 0);
    esp_err_t ret = spi_device_transmit(_spi, &t);
    gpio_set_level(CONFIG_CS_GPIO, 1);
    if (ret != ESP_OK) return -1;
    return in[1];
}

uint16_t crc16(const uint8_t *data, size_t length) {
    uint16_t crc = 0x0000;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

void set_my_mac(void) {
    vTaskDelay(pdMS_TO_TICKS(20));  
    esp_read_mac(MYMAC, ESP_MAC_WIFI_STA);
    vTaskDelay(pdMS_TO_TICKS(20));  

    ESP_LOGI(TAG, "MAC lido: %02X:%02X:%02X:%02X:%02X:%02X",
             MYMAC[0], MYMAC[1], MYMAC[2],
             MYMAC[3], MYMAC[4], MYMAC[5]);

    MY_ID = get_my_id_xor();

    ESP_LOGI(TAG, "ID gerado via XOR dos dois últimos bytes: %d", MY_ID);
}

void send_ack(uint8_t to_id) {
    lora_packet_t ack = {
        .id = MY_ID,
        .dst = to_id,
        .type = LORA_TYPE_ACK,
        .len = 2,
        .payload = { LORA_ACK, to_id },
    };
    ack.crc = crc16((uint8_t *)&ack, sizeof(ack) - sizeof(ack.crc));

    uint8_t *raw = (uint8_t *)&ack;

    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);

    for (int i = 0; i < LORA_PKT_SIZE; i++) {
        lora_write_reg(REG_FIFO, raw[i]);
    }

    lora_write_reg(REG_PAYLOAD_LENGTH, LORA_PKT_SIZE);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    int irq;
    do {
        irq = lora_read_reg(REG_IRQ_FLAGS);
    } while ((irq & IRQ_TX_DONE_MASK) == 0);

    lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    ESP_LOGI(TAG, "ACK enviado: DE=%d, PARA=%d", MY_ID, to_id); 
}


void send_nack(uint8_t to_id) {
    lora_packet_t ack = {
        .id = MY_ID,
        .dst = to_id,
        .type = LORA_TYPE_NACK,
        .len = 2,
        .payload = { LORA_NACK, to_id },
    };
    ack.crc = crc16((uint8_t *)&ack, sizeof(ack) - sizeof(ack.crc));
    
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    lora_write_reg(REG_FIFO_ADDR_PTR, 0);

    uint8_t *raw = (uint8_t *)&ack;
    for (int i = 0; i < LORA_PKT_SIZE; i++) {
        lora_write_reg(REG_FIFO, raw[i]);
    }

    lora_write_reg(REG_PAYLOAD_LENGTH, LORA_PKT_SIZE);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

    int irq;
    do {
        irq = lora_read_reg(REG_IRQ_FLAGS);
    } while ((irq & IRQ_TX_DONE_MASK) == 0);

    lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
    

    ESP_LOGI(TAG, "Enviado: DE=%d, PARA=%d", MY_ID, to_id);    
}



BaseType_t lora_send_raw(lora_packet_t *pkt) {
    ESP_LOGI(TAG, "dentro do lora_send_raw");
    if (xSemaphoreTake(lora_mux, pdMS_TO_TICKS(1000))) {
        //ESP_LOGI(TAG, "peguei o mux");
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
        lora_write_reg(REG_FIFO_ADDR_PTR, 0);

        uint8_t *raw = (uint8_t *)pkt;
        for (int i = 0; i < LORA_PKT_SIZE; i++) {
            lora_write_reg(REG_FIFO, raw[i]);
        }

        lora_write_reg(REG_PAYLOAD_LENGTH, LORA_PKT_SIZE);
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);

        int irq;
        // do {
        //     irq = lora_read_reg(REG_IRQ_FLAGS);
        // } while ((irq & IRQ_TX_DONE_MASK) == 0);
        int retries = 1000;
        do {
            irq = lora_read_reg(REG_IRQ_FLAGS);
            if (retries % 100 == 0) //ESP_LOGW(TAG, "Aguardando TX_DONE... IRQ=0x%02X", irq);
            vTaskDelay(pdMS_TO_TICKS(1));
        } while ((irq & IRQ_TX_DONE_MASK) == 0 && --retries > 0);

        if (retries == 0) {
            //ESP_LOGE(TAG, "Timeout esperando IRQ_TX_DONE. IRQ=0x%02X", irq);
        }

        lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
        lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
        xSemaphoreGive(lora_mux);

        //ESP_LOGI(TAG, "Enviado: ID=%d, payload_len=%d", pkt->id, pkt->len);
        return pdTRUE;
    } else {
        ESP_LOGW(TAG, "lora_send_raw: mutex ocupado, envio abortado.");
        return pdFALSE;
    }
    return pdFALSE;
}

//______________________________________________________________________________________________
//_________________________ Funcoes de inicializacao ___________________________________________
//______________________________________________________________________________________________
// ISR do DIO0 (RxDone)
void IRAM_ATTR dio0_isr_handler(void *arg){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(lora_rx_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Configura interrupção no ESP32 para DIO0
static void setup_dio0_interrupt()
{
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << LORA_DIO0_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en   = 1
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO0_PIN, dio0_isr_handler, NULL);
}

// Inicialização completa do LoRa, retornando 1 em sucesso, 0 em falha
static int lora_init_full()
{
    esp_err_t ret;

    // 1) Configura GPIOs RST e CS
    gpio_reset_pin(CONFIG_RST_GPIO);
    gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_RST_GPIO, 1);

    gpio_reset_pin(CONFIG_CS_GPIO);
    gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_CS_GPIO, 1);

    // 2) Inicializa barramento SPI
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_MISO_GPIO,
        .mosi_io_num = CONFIG_MOSI_GPIO,
        .sclk_io_num = CONFIG_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0
    };
    ret = spi_bus_initialize(HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize falhou: %d", ret);
        return 0;
    }

    // 3) Adiciona dispositivo SPI
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 9 * 1000 * 1000, // 9 MHz
        .mode           = 0,
        .spics_io_num   = CONFIG_CS_GPIO,
        .queue_size     = 4,
        .flags          = 0,
        .pre_cb         = NULL
    };
    ret = spi_bus_add_device(HOST_ID, &devcfg, &_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device falhou: %d", ret);
        return 0;
    }

    // 4) Pulso de reset no LoRa
    lora_reset_pulse();

    // 5) Leitura de REG_VERSION até 0x12 ou timeout
    int version = -1;
    for (int i = 0; i < TIMEOUT_RESET; i++) {
        version = read_lora_version();
        if (version == 0x12) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    if (version != 0x12) {
        ESP_LOGE(TAG, "REG_VERSION inválido: 0x%02X", version);
        return 0;
    }
    ESP_LOGI(TAG, "REG_VERSION = 0x%02X confirmado", version);

    // 6) Configuração padrão pós-checagem de versão
    lora_write_reg(REG_OP_MODE,       MODE_LONG_RANGE_MODE | MODE_SLEEP);
    ESP_LOGI(TAG, "Modo atual: 0x%02X", lora_read_reg(REG_OP_MODE));
    // Ativa LoRa + Standby (imprescindível antes de qualquer operação)
    // lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    // vTaskDelay(pdMS_TO_TICKS(10));  // Delay pequeno para garantir mudança de estado

    uint8_t mode = lora_read_reg(REG_OP_MODE);
    ESP_LOGI(TAG, "Modo após STDBY: 0x%02X", mode);

    lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
    int lna = lora_read_reg(REG_LNA);
    lora_write_reg(REG_LNA, lna | 0x03);
    lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
    lora_write_reg(REG_PA_CONFIG, PA_BOOST | (17 - 2));
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
    return 1;
}



void lora_setup(void) {
    // Inicializa barramento SPI, reset e verifica comunicação com SX1276
    if (!lora_init_full()) {
        ESP_LOGE(TAG, "Falha ao inicializar LoRa");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Configura frequência para 915 MHz (região AU915/TTN BR)
    uint64_t frf = ((uint64_t)(LORA_FREQ_HZ) << 19) / 32000000;
    lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    lora_write_reg(REG_FRF_LSB, (uint8_t)(frf));

    // Ativa CRC interno do chip (verificação física)
    lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);

    // Define parâmetros básicos de modulação: CR=4/5, BW=125kHz, SF=7
    int cr = 1, bw = 7, sf = 7;
    int mc1 = lora_read_reg(REG_MODEM_CONFIG_1);
    lora_write_reg(REG_MODEM_CONFIG_1, (mc1 & 0xF1) | (cr << 1) | (bw << 4));

    int mc2 = lora_read_reg(REG_MODEM_CONFIG_2);
    lora_write_reg(REG_MODEM_CONFIG_2, (mc2 & 0x0F) | (sf << 4));

    ESP_LOGI(TAG, "Parâmetros: CR=%d, BW=%d, SF=%d", cr, bw, sf);

    // Mapeia DIO0 para gerar interrupção quando RxDone
    lora_write_reg(REG_DIO_MAPPING_1, 0x00);

    // Cria mutex para controle de acesso ao LoRa
    lora_mux = xSemaphoreCreateMutex();

    // Configura a interrupção externa do DIO0
    setup_dio0_interrupt();

    xTaskCreatePinnedToCore(task_lora_rx, "lora_rx", 4096, NULL, 5, &lora_rx_task_handle, 0);
    // Define MAC e ID baseado no endereço físico
    set_my_mac();

    ESP_LOGI(TAG, "LoRa configurado e pronto.");
}


void lora_handle_packet(lora_packet_t *pkt) {
    if (packet_handler != NULL)
        packet_handler(pkt);  // callback do usuário
    else
        lora_default_handler(pkt);  // fallback com switch
}

void lora_default_handler(lora_packet_t *pkt) {
    switch (pkt->type) {
        case LORA_TYPE_DATA:
            ESP_LOGI("DEFAULT_HANDLER", "Dado recebido: %.*s", pkt->len, pkt->payload);
            break;

        case LORA_TYPE_ACK:
            ESP_LOGI("DEFAULT_HANDLER", "ACK recebido de %d", pkt->id);
            break;

        case LORA_TYPE_NACK:
            ESP_LOGW("DEFAULT_HANDLER", "NACK recebido de %d", pkt->id);
            break;

        default:
            ESP_LOGW("DEFAULT_HANDLER", "Tipo desconhecido: 0x%02X", pkt->type);
            break;
    }
}


void task_lora_rx(void *pvParameters)
{
    uint8_t buf[256];
    lora_packet_t pkt;

    // Inicia modo RX contínuo
    lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(lora_mux, pdMS_TO_TICKS(100)) == pdTRUE) {
            int irq = lora_read_reg(REG_IRQ_FLAGS);

            if (irq & IRQ_RX_DONE_MASK) {
                int len = lora_read_reg(REG_RX_NB_BYTES);
                if (len > sizeof(buf)) len = sizeof(buf);

                int curr = lora_read_reg(REG_FIFO_RX_CURRENT_ADDR);
                lora_write_reg(REG_FIFO_ADDR_PTR, curr);

                for (int i = 0; i < len; i++) {
                    buf[i] = lora_read_reg(REG_FIFO);
                }

                // Confere se tamanho bate com o esperado
                if (len == LORA_PKT_SIZE) {
                    memcpy(&pkt, buf, sizeof(pkt));
                    uint16_t crc_calc = crc16((uint8_t*)&pkt, sizeof(pkt) - sizeof(pkt.crc));

                    if (crc_calc == pkt.crc) {
                        if (pkt.dst == MY_ID && pkt.id == ID_DESTINO) {
                            lora_handle_packet(&pkt);
                            
                        } else {
                            ESP_LOGI(TAG, "Pacote ignorado (dst=%d, eu sou %d)", pkt.dst, MY_ID);
                        }
                    } else {
                        ESP_LOGW(TAG, "CRC inválido! esperado=0x%04X, recebido=0x%04X", crc_calc, pkt.crc);
                    }
                } else {
                    ESP_LOGW(TAG, "Tamanho inesperado: %d (esperado: %d)", len, LORA_PKT_SIZE);
                }
            }

            // Limpa flags e volta ao modo RX contínuo
            lora_write_reg(REG_IRQ_FLAGS, irq);
            lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
            // int rssi = lora_read_reg(0x1A) - 137;
            // int snr  = (int8_t)lora_read_reg(0x19) / 4;
            // ESP_LOGI(TAG, "Sinal: RSSI=%d dBm, SNR=%d dB", rssi, snr);
            xSemaphoreGive(lora_mux);
        }
    }

}

//______________________________________________________________________________________________
//_________________________ Funcoes de uso______________________________________________________
//______________________________________________________________________________________________


BaseType_t lora_send_structured(const char *msg, uint8_t dest_id)
{
    int len = strlen(msg);
    if (len > LORA_PAYLOAD_MAX_LEN) len = LORA_PAYLOAD_MAX_LEN;

    lora_packet_t pkt = {
        .id = MY_ID,
        .dst = dest_id, 
        .type = LORA_TYPE_DATA,
        .len = len,
    };
    memcpy(pkt.payload, msg, len);
    pkt.crc = crc16((uint8_t*)&pkt, sizeof(pkt) - sizeof(pkt.crc));

    if(lora_send_raw(&pkt))
        return pdTRUE;
    else 
        return pdFALSE;
}


