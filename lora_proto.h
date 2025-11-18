#pragma once
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lora.h"
#include "esp_timer.h"
#include "esp_mac.h"

//----------------------------- COMANDOS LORA -----------------------------
#define REG_OP_MODE             0x01
#define MODE_LONG_RANGE_MODE    0x80
#define MODE_SLEEP              0x00
#define REG_FIFO_RX_BASE_ADDR   0x0F
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_LNA                 0x0C
#define REG_MODEM_CONFIG_3      0x26
#define REG_PA_CONFIG           0x09
#define PA_BOOST                0x80
#define MODE_STDBY              0x01
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO                0x00
#define REG_PAYLOAD_LENGTH      0x22
#define MODE_TX                 0x03
#define REG_IRQ_FLAGS           0x12
#define IRQ_TX_DONE_MASK        0x08
#define MODE_RX_CONTINUOUS      0x05
#define IRQ_RX_DONE_MASK        0x40
#define REG_RX_NB_BYTES         0x13
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_MODEM_CONFIG_2      0x1E
#define REG_MODEM_CONFIG_1      0x1D
#define REG_DIO_MAPPING_1       0x40

//----------------------------- PINOUT -----------------------------------

#ifndef CONFIG_MOSI_GPIO
#define CONFIG_MOSI_GPIO        27
#endif
#ifndef CONFIG_MISO_GPIO
#define CONFIG_MISO_GPIO        19
#endif
#ifndef CONFIG_SCK_GPIO
#define CONFIG_SCK_GPIO         5
#endif
#ifndef CONFIG_CS_GPIO
#define CONFIG_CS_GPIO          18
#endif
#ifndef CONFIG_RST_GPIO
#define CONFIG_RST_GPIO         14
#endif
#define LORA_DIO0_PIN           GPIO_NUM_26

//------------------------- DEFINIÇÕES -----------------------------------
#define LORA_TYPE_DATA 0x01
#define LORA_TYPE_ACK  0x02
#define LORA_TYPE_NACK 0x03 

#define LORA_ACK       0xAC
#define LORA_NACK      0x5A

#define HOST_ID        SPI2_HOST
#define LORA_FREQ_HZ   915000000ULL
#define TIMEOUT_RESET  100

#define LORA_PKT_SIZE sizeof(lora_packet_t)
#define LORA_PAYLOAD_MAX_LEN 48
//------------------------- ESTRUTURAS -----------------------------------
typedef struct {
    uint8_t id;
    uint8_t dst;
    uint8_t type;
    uint8_t len;
    uint8_t payload[48];
    uint16_t crc;
} __attribute__((packed)) lora_packet_t;

//------------------------- VARIÁVEIS GLOBAIS ----------------------------
static uint8_t MYMAC[6]; 
extern uint8_t ID_DESTINO;


//------------------------- FUNÇÕES PÚBLICAS -----------------------------
uint16_t crc16(const uint8_t *data, size_t length);

void set_my_mac(void);
typedef void (*lora_packet_handler_t)(lora_packet_t *pkt);
void send_ack(uint8_t to_id);
void send_nack(uint8_t to_id);
BaseType_t lora_send_raw(lora_packet_t *pkt);
BaseType_t lora_send_structured(const char *msg, uint8_t dest_id);
void lora_default_handler(lora_packet_t *pkt);
void lora_set_packet_handler(lora_packet_handler_t handler);
void lora_handle_packet(lora_packet_t *pkt);
void task_lora_rx(void *pvParameters);
void lora_setup(void);

//------------------------- INLINE ----------------------------------------
static inline uint8_t get_my_id_xor(void) {
    return MYMAC[4] ^ MYMAC[5];
}
