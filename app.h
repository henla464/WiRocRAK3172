#include <Arduino.h>

#define LED_RED_TRANSMIT PIN_LED1
#define LED_BLUE_RECEIVE PIN_LED2
#define LORA_AUX PB5
#define LORA_IRQ PA8
#define MSG_TYPE_PUNCH  0x07
#define MSG_TYPE_PUNCH_DOUBLE  0x08

// Forward declarations
bool init_status_at(void);
bool init_send_at(void);
bool init_config_at(void);
bool init_receive_at(void);