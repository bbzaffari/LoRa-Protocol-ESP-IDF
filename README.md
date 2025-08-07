# LoRa-Proto 

[![Status: stable | refactoring in progress](https://img.shields.io/badge/Status-stable%20%7C%20refactoring%20in%20progress-green.svg)](https://github.com/bbzaffari/Cooling-Link-Controller)

# LoRa Protocol Layer – High-Level Overview

This module implements a custom protocol layer over the LoRa transceiver (SX1276), using ESP32 + FreeRTOS. The logic is organized modularly, with SPI low-level control functions encapsulated and separated from packet handling and protocol abstraction.

---

## General Structure

- **`lora_setup()`** – Initializes the SPI bus, configures the LoRa chip, sets physical modulation parameters (SF, BW, CR), maps interrupts, and creates the reception task (`task_lora_rx`).
- **`task_lora_rx()`** – A dedicated task for packet reception, running with **high priority** (level 5) and pinned to core 0. It is triggered via an ISR associated with the DIO0 pin (`RxDone` event) and handles reading, validating (CRC and address), and dispatching the packet.

---

## Packet Handling

The application can define its own callback to process incoming packets:

```c
typedef void (*lora_packet_handler_t)(lora_packet_t *pkt);
void lora_set_packet_handler(lora_packet_handler_t handler);

