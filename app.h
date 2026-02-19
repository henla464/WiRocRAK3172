#include <Arduino.h>

#define MYLOG(tag, ...)                  \
	do                                   \
	{                                    \
		if (tag)                         \
			Serial.printf("[%s] ", tag); \
		Serial.printf(__VA_ARGS__);      \
		Serial.printf("\n");             \
	} while (0);                         \
	delay(100)

#define LED_RED_TRANSMIT PIN_LED1
#define LED_BLUE_RECEIVE PIN_LED2

/** Custom flash parameters structure */
struct custom_param_s
{
	uint8_t valid_flag = 0xAA;
	uint32_t send_interval = 0;
};

// Forward declarations
bool init_status_at(void);
bool init_send_at(void);
bool init_config_at(void);
bool init_receive_at(void);