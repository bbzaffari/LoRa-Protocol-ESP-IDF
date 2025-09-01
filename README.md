# LoRa Protocol Layer 

[![Status: stable | refactoring in progress](https://img.shields.io/badge/Status-stable%20%7C%20refactoring%20in%20progress-green.svg)](https://github.com/bbzaffari/Cooling-Link-Controller)

## Overview

This module implements a custom protocol layer over the LoRa (SX1276), using ESP32 + FreeRTOS. The logic is organized modularly, with SPI low-level control functions encapsulated and separated from packet handling and protocol abstraction.


## General Structure

- **`lora_setup()`** – Initializes the SPI bus, configures the LoRa chip, sets physical modulation parameters (SF, BW, CR), maps interrupts, and creates the reception task (`task_lora_rx`).
- **`task_lora_rx()`** – A dedicated task for packet reception, running with **high priority** (level 5) and pinned to core 0. It is triggered via an ISR associated with the DIO0 pin (`RxDone` event) and handles reading, validating (CRC and address), and dispatching the packet.
  
---
---

## Packet Handling

The application can define its own callback to process incoming packets:

```c
typedef void (*lora_packet_handler_t)(lora_packet_t *pkt);
void lora_set_packet_handler(lora_packet_handler_t handler);
```

```c
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

                // Check if the size matches what is expected
                if (len == LORA_PKT_SIZE) {
                    memcpy(&pkt, buf, sizeof(pkt));
                    uint16_t crc_calc = crc16((uint8_t*)&pkt, sizeof(pkt) - sizeof(pkt.crc));

                    if (crc_calc == pkt.crc) {
                        if (pkt.dst == MY_ID && pkt.id == ID_DESTINO) {
                           lora_handle_packet(&pkt);     //  <<-------------------------------
                            
                        } else {
                            ESP_LOGI(TAG, "Package ignored (dst: %d, I am: %d)", pkt.dst, MY_ID);
                        }
                    } else {
                        ESP_LOGW(TAG, "Invalid CRC! Expected:=0x%04X, received=0x%04X", crc_calc, pkt.crc);
                    }
                } else {
                    ESP_LOGW(TAG, "Unexpected size: %d (Expected: %d)", len, LORA_PKT_SIZE);
                }
            }

            // Clear flags and return to continuous RX mode
            lora_write_reg(REG_IRQ_FLAGS, irq);
            lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
            // int rssi = lora_read_reg(0x1A) - 137;
            // int snr  = (int8_t)lora_read_reg(0x19) / 4;
            // ESP_LOGI(TAG, "Sinal: RSSI=%d dBm, SNR=%d dB", rssi, snr);
            xSemaphoreGive(lora_mux);
        }
    }

}
````
### Example of APPLICATION RECEIVER HANDLER
```c
static void handler_LoRa_Rx_Controler(lora_packet_t *pkt) {

    if ((pkt->type) == LORA_TYPE_DATA){   
        char TAG[] = "RX LORA_TYPE_DATA";
        ESP_LOGI(TAG, "Received: %.*s", pkt->len, pkt->payload); // DEBUG
        /*
        * Logic
        */
    }

    else if((pkt->type) == LORA_TYPE_ACK) {
        ACK_PENDENTE = pdFALSE; 
        ESP_LOGI("RX LORA_TYPE_ACK", "ACK received from %d", pkt->id); // DEBUG
    }

    else if ((pkt->type) ==LORA_TYPE_NACK){
        ESP_LOGW("RX LORA_TYPE_NACK", "NACK received from %d", pkt->id); // DEBUG
    }      
}
````
