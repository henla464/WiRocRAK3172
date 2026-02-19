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

    pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, LOW);
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, LOW);

    // Delay for 5 seconds to give the chance for AT+BOOT
	delay(5000);
	digitalWrite(LED_GREEN, HIGH);
	digitalWrite(LED_BLUE, HIGH);
    // AT+LPM=1  AT+LPMLVL=2

    // Register the custom AT command to get device status
	if (!init_status_at())
	{
		MYLOG("SETUP", "Add custom AT command STATUS fail");
	} else {
        MYLOG("SETUP", "Custom AT command STATUS initialized");
    }
    
    if (!init_config_at())
	{
		MYLOG("SETUP", "Add custom AT command P2P Config fail");
	} else {
        MYLOG("SETUP", "Custom AT command P2P Config initialized");
    }

    if (!init_send_at())
	{
		MYLOG("SETUP", "Add custom AT command SEND fail");
	} else {
        MYLOG("SETUP", "Custom AT command SEND initialized");
    }

    if (!init_receive_at())
	{
		MYLOG("SETUP", "Add custom AT command REC fail");
	} else {
        MYLOG("SETUP", "Custom AT command REC initialized");
    }

    // Enable RX permanent with TX possible
	api.lora.precv(65533);

	// Enable low power mode
	//api.system.lpm.set(2);
   
}

void loop() {
    //Serial.println("Hello from RAK3172!");
    delay(1000);
}


