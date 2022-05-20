// #define USE_WIFI
#include <Arduino.h>

#include "gooseka_defs.h"
#include "gooseka_structs.h"
#include "gooseka_helpers.h"
#include "gooseka_lora.h"

#define STATE_SOF_1 0x00
#define STATE_SOF_2 0x01
#define STATE_FRAME 0x02

#define SOF_1 0xDE
#define SOF_2 0xAD

QueueHandle_t control_queue;

// CPU #1
void radio_receive_task(void* param) {
    uint8_t radio_buffer[sizeof(ESC_telemetry_t)];
    // uint8_t index;
    ESC_control_t control;
    long last_sent_millis = 0;

    memset(&control,0,sizeof(ESC_control_t));

    while(1) {
        int packetSize = receive_radio_packet(radio_buffer, sizeof(ESC_telemetry_t));
        if (packetSize == sizeof(ESC_telemetry_t)) {
            // Send packet via USB
            Serial.write(SOF_1);
            Serial.write(SOF_2);
            Serial.write(radio_buffer, sizeof(ESC_telemetry_t));
            // Serial.write(radio_rssi());
        }

        if(xQueueReceive(control_queue, &control, 0)) {
            if(millis() - last_sent_millis > LORA_SLOWDOWN) {
                // Send enqueued radio msgs
                if(control.magic_number == MAGIC_NUMBER) {
                    send_via_radio((uint8_t *)&control, sizeof(ESC_control_t));
                }
                last_sent_millis = millis();
            }
        }

        vTaskDelay(1); // Without this line watchdog resets the board
    }     
    vTaskDelete(NULL);
}

// CPU #0
void USB_receive_task(void* param) {
    uint8_t state = STATE_SOF_1;
    uint8_t index;
    uint8_t buffer[sizeof(ESC_control_t)];
    memset(&buffer, 0, sizeof(ESC_control_t));

    while(1){
        if (Serial.available() > 0) {
            // Receive USB char
            uint8_t incomingByte = Serial.read();
            // DEBUG_PRINTLN(incomingByte, HEX);
            switch(state) {
                case STATE_SOF_1:
                    // DEBUG_PRINTLN("SOF_1");
                    if(incomingByte == SOF_1) {
                        state = STATE_SOF_2;
                    }
                break;
                case STATE_SOF_2:
                    // DEBUG_PRINTLN("SOF_2");
                    if(incomingByte == SOF_2) {
                        state = STATE_FRAME;
                        index = 0;
                    } else {
                        state = STATE_SOF_1;
                    }
                break;
                case STATE_FRAME:
                    // DEBUG_PRINTLN("FRAME");
                    if (index < sizeof(ESC_control_t) - 1) {
                        buffer[index] = incomingByte;
                        index++;
                    } else {
                        buffer[index] = incomingByte;
                        state = STATE_SOF_1;
                        // DEBUG_PRINTLN("SENDING");
                        xQueueSend(control_queue, buffer, 0);
                    }
                break;
                default:
                    state = STATE_SOF_1;
                break;
            }
        }
        vTaskDelay(1); // Without this line watchdog resets the board
    }
    vTaskDelete(NULL);
}

void setup() {
    // Initialize structs and arrays
    pinMode(LED_BUILTIN, OUTPUT);

    // USB output
    Serial.begin(SERIAL_BAUDRATE);

    // Init control msg queue
    control_queue = xQueueCreate(QUEUE_SIZE, sizeof(ESC_control_t));

    // Initialize radio interface
    init_radio();

    // Start USB receiver task
    xTaskCreatePinnedToCore(USB_receive_task, "USB_receiver", 10000, NULL, 1, NULL, 0);
    // Start LoRa receiver task
    xTaskCreatePinnedToCore(radio_receive_task, "radio_receiver", 10000, NULL, 1, NULL, 1);
}

void loop() {
    delay(2147483647L); // Delay forever
}
