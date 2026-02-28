#include <Arduino.h>
#include "app.h"

void setup() {
    Serial1.begin(115200, RAK_AT_MODE);
    Serial.begin(115200, RAK_AT_MODE);
        
    // Set to P2P mode
    if(api.lora.nwm.get() != 0)
    {
        api.lora.nwm.set();
        api.system.reboot();
    }

    // nothing being sent
    pinMode(LORA_AUX, OUTPUT);
    digitalWrite(LORA_AUX, HIGH);
    // nothing in receive queue
    pinMode(LORA_IRQ, OUTPUT);
    digitalWrite(LORA_IRQ, LOW);

    // Register the custom AT command to get device status
	if (!init_status_at())
	{
		Serial.printf("SETUP Add custom AT command STATUS fail\r\n");
	} else {
        Serial.printf("SETUP Custom AT command STATUS initialized\r\n");
    }
    
    if (!init_config_at())
	{
		Serial.printf("SETUP Add custom AT command P2P Config fail\r\n");
	} else {
        Serial.printf("SETUP Custom AT command P2P Config initialized\r\n");
    }

    if (!init_send_at())
	{
		Serial.printf("SETUP Add custom AT command SEND fail\r\n");
	} else {
        Serial.printf("SETUP Custom AT command SEND initialized\r\n");
    }

    if (!init_receive_at())
	{
		Serial.printf("SETUP Add custom AT command REC fail\r\n");
	} else {
        Serial.printf("SETUP Custom AT command REC initialized\r\n");
    }

    // Enable RX permanent with TX possible
	api.lora.precv(65533);

    pinMode(LED_RED_TRANSMIT, OUTPUT);
	digitalWrite(LED_RED_TRANSMIT, LOW);
	pinMode(LED_BLUE_RECEIVE, OUTPUT);
	digitalWrite(LED_BLUE_RECEIVE, LOW);
    // Delay for 4 seconds to give the chance for AT+BOOT
	delay(4000);
	digitalWrite(LED_RED_TRANSMIT, HIGH);
	digitalWrite(LED_BLUE_RECEIVE, HIGH);

	// Enable low power mode
	api.system.lpm.set(1);
    // can't select level 2 because level 2 doesn't
    // support wakeing up for UART1 AT command
    api.system.lpmlvl.set(1);   
}

void loop() {
    //Serial.println("Hello from RAK3172!");
    //delay(1000);
    api.system.scheduler.task.destroy();
}


