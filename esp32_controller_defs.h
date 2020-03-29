#ifndef ESP32_CONTROLLER_DEFS_H
#define ESP32_CONTROLLER_DEFS_H

// LORA TRANSCEIVER PINS
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// LORA FREQUENCY BAND
// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define LORA_BAND 866E6
#define LORA_SPREADING_FACTOR 7
#define LORA_BANDWIDTH 250e3
#define LORA_CODING_RATE 5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_TX_POWER 20

// LORA SYNCWORD
#define LORA_SYNCWORD 0xCA

// USB SERIAL SPEED (7.3728*1e6/2^N)
// #define SERIAL_BAUDRATE 115200
#define SERIAL_BAUDRATE 230400
// #define SERIAL_BAUDRATE 460800
// #define SERIAL_BAUDRATE 921600
// #define SERIAL_BAUDRATE 1843200

// INTER CPU MESG QUEUE SIZE
#define QUEUE_SIZE 1

// LORA SENDER SLOW DOWN 
// (do not send more than one msg every LORA_SLOWDOWN ms)
#define LORA_SLOWDOWN 100L

// MAGIC NUMBER TO CHECK FOR USB ERRORS
#define MAGIC_NUMBER 0xCA

#endif /* ESP32_CONTROLLER_DEFS_H */
