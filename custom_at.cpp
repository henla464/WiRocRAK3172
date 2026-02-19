
#include "app.h"
#include "radio.h"
#include "radio_driver.h"
#include "service_runtimeConfig.h"
#include "service_lora_p2p.h"
#include "MessageQueue.h"

/** Custom flash parameters */
#define AT_PRINTF(sPort, ...)              \
	do                              \
	{                               \
		sPort->printf(__VA_ARGS__); \
		sPort->printf("\r\n");      \
	} while (0);                    \
	delay(100)


int status_handler(SERIAL_PORT port, char *cmd, stParam *param);
int send_handler(SERIAL_PORT port, char *cmd, stParam *param);
int config_handler(SERIAL_PORT port, char *cmd, stParam *param);
int receive_handler(SERIAL_PORT port, char *cmd, stParam *param);
void send_cb(void);
void cad_cb(bool detect);
void receive_cb(rui_lora_p2p_recv_t recv_data_pkg);


bool init_status_at(void)
{
	return api.system.atMode.add((char *)"STATUS",
								 (char *)"Get device information",
								 (char *)"STATUS", status_handler,
								 RAK_ATCMD_PERM_READ);
}

bool init_config_at(void)
{

	return api.system.atMode.add((char *)"P2P",
								 (char *)"Configure in P2P mode. Usage: ATC+P2P=<frequency>:<spreading factor>:<bandwidth>:<coding rate>:<preamble length>:<tx power>:<low data rate optimize>:<crc on>:<rx gain>:<payload length>",
								 (char *)"P2P", config_handler,
								 RAK_ATCMD_PERM_READ | RAK_ATCMD_PERM_WRITE);
}

bool init_send_at(void)
{

	api.lora.registerPSendCallback(send_cb);
	api.lora.registerPSendCADCallback(cad_cb);
	return api.system.atMode.add((char *)"SEND",
								 (char *)"Send data in P2P mode. Usage: ATC+SEND=<payload>",
								 (char *)"SEND", send_handler,
								 RAK_ATCMD_PERM_WRITE);
}

bool init_receive_at(void)
{
    api.lora.registerPRecvCallback(receive_cb);
	return api.system.atMode.add((char *)"REC",
								 (char *)"Get a received message. Usage ATC+REC or ATC+REC=? to get data in binary (first two chars OK). ATC+REC=H to get data returned in hex. \
                        Returned message 'OK<binary data, rssi, snr, status>' or 'OK<hex data>:rssi:snr:status'. When no data to fetch 'EM' is returned",
								 (char *)"REC", receive_handler,
								 RAK_ATCMD_PERM_READ| RAK_ATCMD_PERM_WRITE);
}

// STATUS //

/** Network modes as text array*/
char *nwm_list[] = {"P2P", "LoRaWAN", "FSK"};

/**
 * @brief Print device status over Serial
 *
 * @param port Serial port used
 * @param cmd char array with the received AT command
 * @param param char array with the received AT command parameters
 * @return int result of command parsing
 * 			AT_OK AT command & parameters valid
 * 			AT_PARAM_ERROR command or parameters invalid
 */
int status_handler(SERIAL_PORT port, char *cmd, stParam *param)
{
	String value_str = "";
	int nw_mode = 0;
    HardwareSerial* serialX;
    if (port == SERIAL_UART1)
    {
        serialX = &Serial1;
    }
    else if (port == SERIAL_UART2)
    {
        serialX = &Serial;
    }

    if ((param->argc == 1 && !strcmp(param->argv[0], "?")) || (param->argc == 0))
	{
		AT_PRINTF(serialX, "Device Status:");
		value_str = api.system.hwModel.get();
		value_str.toUpperCase();
		AT_PRINTF(serialX, "Module: %s", value_str.c_str());
		AT_PRINTF(serialX, "Version: %s", api.system.firmwareVer.get().c_str());
        nw_mode = api.lora.nwm.get();
		AT_PRINTF(serialX, "Network mode %s", nwm_list[nw_mode]);
        AT_PRINTF(serialX, "Frequency = %d", api.lora.pfreq.get());
        AT_PRINTF(serialX, "SF = %d", api.lora.psf.get());
        AT_PRINTF(serialX, "BW = %d", api.lora.pbw.get());
        AT_PRINTF(serialX, "CR = %d", api.lora.pcr.get());
        AT_PRINTF(serialX, "Preamble length = %d", api.lora.ppl.get());
        AT_PRINTF(serialX, "TX power = %d", api.lora.ptp.get());
        AT_PRINTF(serialX, "Low datarate optimization = %s", api.lora.lowDataRateOptimize.get() ? "ON" : "OFF");
        AT_PRINTF(serialX, "CRC = %s", api.lora.crc.get() ? "ON" : "OFF");
        AT_PRINTF(serialX, "Rx Gain %s", api.lora.rxgain.get() ? "ON" : "OFF");
        AT_PRINTF(serialX, "Fixed length payload = %s", api.lora.fixLengthPayload.get() ? "ON" : "OFF");
		AT_PRINTF(serialX, "Payload length = %d", api.lora.payloadLength.get());
	}
	else 
	{
		return AT_PARAM_ERROR;
	}
	return AT_OK;
}


/**
 * @brief Configures the P2P parameters of the device
 * Including lowDataRateOptimization, crc and payload length.
 *
 * @param port Serial port used
 * @param cmd char array with the received AT command
 * @param param char array with the received AT command parameters
 * @return int result of command parsing
 * 			AT_OK AT command & parameters valid
 * 			AT_PARAM_ERROR command or parameters invalid
 * 
 * Note: This is almost a copy of the orginal AT config command, only with some added paramters.
 * Original found in RAK-STM32-RUI/cores/STM32WLE/component/service/mode/cli/atcmd_p2p.c
 */
int config_handler(SERIAL_PORT port, char *cmd, stParam *param)
{
    HardwareSerial* serialX;
    /*
    if (port == SERIAL_UART1)
    {
        serialX = &Serial1;
    }
    else if (port == SERIAL_UART2)
    {
        serialX = &Serial;
    }
    AT_PRINTF(serialX, "argc %d", param->argc);
    AT_PRINTF(serialX, "arcv %s", param->argv[0]);
    */
    if (param->argc == 1 && !strcmp(param->argv[0], "?"))
    {
        if (get_useRuntimeConfigP2P())
        {
            runtimeConfigP2P_t runtimeConfigP2P;
            get_runtimeConfigP2P(&runtimeConfigP2P);
            atcmd_printf("%s=", cmd);
            atcmd_printf("%u:", runtimeConfigP2P.frequency);
            atcmd_printf("%u:", runtimeConfigP2P.spreading_factor);
            atcmd_printf("%u:", runtimeConfigP2P.bandwidth);  
            atcmd_printf("%u:", runtimeConfigP2P.coding_rate);
            atcmd_printf("%u:", runtimeConfigP2P.preamble_length);
            atcmd_printf("%u:", runtimeConfigP2P.txpower);
			atcmd_printf("%d:", runtimeConfigP2P.low_data_rate_optimize);
			atcmd_printf("%d:", runtimeConfigP2P.crc_on);
            atcmd_printf("%d:", runtimeConfigP2P.rxgain);
			atcmd_printf("%u", runtimeConfigP2P.payload_len);
        }
        else
	   	{
            atcmd_printf("%s=", cmd);
            atcmd_printf("%u:", service_lora_p2p_get_freq());
            atcmd_printf("%u:", service_lora_p2p_get_sf());
            atcmd_printf("%u:", service_lora_p2p_get_bandwidth());
            atcmd_printf("%u:", service_lora_p2p_get_codingrate());
            atcmd_printf("%u:", service_lora_p2p_get_preamlen());
            atcmd_printf("%u:", service_lora_p2p_get_powerdbm());
			atcmd_printf("%d:", service_lora_p2p_get_low_datarate_optimize());
			atcmd_printf("%d:", service_lora_p2p_get_crcon());
            atcmd_printf("%d:", service_lora_p2p_get_rxgain());
			atcmd_printf("%u", service_lora_p2p_get_payloadlen());
        }
        return AT_NO_STATUS;
    }
    else if (param->argc == 10 || (param->argc == 11 && !strcmp(param->argv[10],"0")))
    {
        uint32_t frequency,spreading_factor,bandwidth,coding_rate,preamble_length,
			txpower, low_data_rate_optimize, crc_on, rxgain, payload_len;
		bool b_crc_on, b_rxgain, b_low_data_rate_optimize;
        uint32_t o_frequency,o_spreading_factor,o_bandwidth,o_coding_rate,
			o_preamble_length,o_txpower;
		bool o_low_data_rate_optimize, o_crc_on, o_rxgain, o_fix_length_payload;
		uint8_t o_payload_len;
        uint8_t udrv_code;

        // Preserve current p2p parameters
        o_frequency = service_lora_p2p_get_freq();
        o_spreading_factor = service_lora_p2p_get_sf();
        o_bandwidth = service_lora_p2p_get_bandwidth();
        o_coding_rate = service_lora_p2p_get_codingrate();
        o_preamble_length = service_lora_p2p_get_preamlen();
        o_txpower = service_lora_p2p_get_powerdbm();
		o_low_data_rate_optimize = service_lora_p2p_get_low_datarate_optimize();
		o_crc_on = service_lora_p2p_get_crcon();
        o_rxgain = service_lora_p2p_get_rxgain();
		o_payload_len = service_lora_p2p_get_payloadlen();
		o_fix_length_payload = service_lora_p2p_get_fix_length_payload();

        // Exchange parameters
        if (0 != at_check_digital_uint32_t(param->argv[0], &frequency))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[1], &spreading_factor))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[2], &bandwidth))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[3], &coding_rate))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[4], &preamble_length))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[5], &txpower))
            return AT_PARAM_ERROR;
		if (0 != at_check_digital_uint32_t(param->argv[6], &low_data_rate_optimize))
            return AT_PARAM_ERROR;
		if (0 != at_check_digital_uint32_t(param->argv[7], &crc_on))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[8], &rxgain))
            return AT_PARAM_ERROR;
		if (0 != at_check_digital_uint32_t(param->argv[9], &payload_len))
            return AT_PARAM_ERROR;


        if ((frequency < 150e6) || (frequency > 960e6))
            return AT_PARAM_ERROR;

        if ( spreading_factor < 5 || spreading_factor > 12)
            return AT_PARAM_ERROR;

        if (coding_rate > 3)
            return AT_PARAM_ERROR;

        if(preamble_length< 5 || preamble_length > 65535)
            return AT_PARAM_ERROR;

        if (txpower < 5 || txpower > 22)
            return AT_PARAM_ERROR;

		if (crc_on > 0)
			b_crc_on = true;
		else
			b_crc_on = false;
        
        if (rxgain > 0)
			b_rxgain = true;
		else
			b_rxgain = false;
		
		if (low_data_rate_optimize > 0)
			b_low_data_rate_optimize = true;
		else			
			b_low_data_rate_optimize = false;	

		if (payload_len > 255)
            return AT_PARAM_ERROR;

        // Can only configure when recieve mode not enabled
        api.lora.precv(0);

        // Check and save parameters
        udrv_code = service_lora_p2p_set_freq(frequency);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_set_sf((uint8_t)spreading_factor);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_set_bandwidth(bandwidth);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_set_codingrate((uint8_t)coding_rate);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_set_preamlen((uint16_t)preamble_length);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_set_powerdbm((uint8_t)txpower);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
		udrv_code = service_lora_p2p_set_low_datarate_optimize(b_low_data_rate_optimize);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
		udrv_code = service_lora_p2p_set_crcon(b_crc_on);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_set_rxgain(b_rxgain);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
		udrv_code = service_lora_p2p_set_payloadlen(payload_len);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;
		udrv_code = service_lora_p2p_set_fix_length_payload(payload_len > 0);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P_CHECK_ERROR_CODE;

        set_useRuntimeConfigP2P(false);
        api.lora.precv(65533);
        return AT_OK;

        STEP_ATP2P_CHECK_ERROR_CODE:
        //Restore the previous parameters
        service_lora_p2p_set_freq(o_frequency);
        service_lora_p2p_set_sf((uint8_t)o_spreading_factor);
        service_lora_p2p_set_bandwidth(o_bandwidth);
        service_lora_p2p_set_codingrate((uint8_t)o_coding_rate);
        service_lora_p2p_set_preamlen((uint16_t)o_preamble_length);
        service_lora_p2p_set_powerdbm((uint8_t)o_txpower);
		service_lora_p2p_set_low_datarate_optimize(o_low_data_rate_optimize);
		service_lora_p2p_set_crcon(o_crc_on);
        service_lora_p2p_set_rxgain(o_rxgain);
		service_lora_p2p_set_payloadlen((uint8_t)o_payload_len);
		service_lora_p2p_set_fix_length_payload(o_fix_length_payload);
		
        api.lora.precv(65533);
        //Check and return error code
        return at_error_code_form_udrv(udrv_code);
    }
    else if (param->argc == 11 && !strcmp(param->argv[10],"1")) { //for runtime setting
        uint32_t frequency,spreading_factor,bandwidth,coding_rate,preamble_length,
			txpower, low_data_rate_optimize, crc_on, rxgain, payload_len;
        uint32_t o_frequency,o_spreading_factor,o_bandwidth,o_coding_rate,o_preamble_length,o_txpower;
		bool o_low_data_rate_optimize, o_crc_on, o_rxgain, o_fix_length_payload;
		uint8_t o_payload_len;
        uint8_t udrv_code;
        bool o_useRuntimeConfig = get_useRuntimeConfigP2P();
        runtimeConfigP2P_t runtimeConfigP2P;

        // Preserve current p2p parameters
        if (o_useRuntimeConfig) {
            get_runtimeConfigP2P(&runtimeConfigP2P);
            o_frequency = runtimeConfigP2P.frequency;
            o_spreading_factor = runtimeConfigP2P.spreading_factor;
            if (SERVICE_LORA_P2P == service_lora_p2p_get_nwm()) {
                o_bandwidth = runtimeConfigP2P.bandwidth;
            }
            else if (SERVICE_LORA_FSK == service_lora_p2p_get_nwm()) {
                o_bandwidth = runtimeConfigP2P.fsk_rxbw;
            }
            o_coding_rate = runtimeConfigP2P.coding_rate;
            o_preamble_length = runtimeConfigP2P.preamble_length;
            o_txpower = runtimeConfigP2P.txpower;
			o_low_data_rate_optimize = runtimeConfigP2P.low_data_rate_optimize;
			o_crc_on = runtimeConfigP2P.crc_on;
            o_rxgain = runtimeConfigP2P.rxgain;
			o_payload_len = runtimeConfigP2P.payload_len;
			o_fix_length_payload = runtimeConfigP2P.fix_length_payload;
        }
        else {
            o_frequency = service_lora_p2p_get_freq();
            o_spreading_factor = service_lora_p2p_get_sf();
            o_bandwidth = service_lora_p2p_get_bandwidth();
            o_coding_rate = service_lora_p2p_get_codingrate();
            o_preamble_length = service_lora_p2p_get_preamlen();
            o_txpower = service_lora_p2p_get_powerdbm();
			o_low_data_rate_optimize = service_lora_p2p_get_low_datarate_optimize();
			o_crc_on = service_lora_p2p_get_crcon();
            o_rxgain = service_lora_p2p_get_rxgain();
			o_payload_len = service_lora_p2p_get_payloadlen();
			o_fix_length_payload = service_lora_p2p_get_fix_length_payload();
        }

        // Exchange parameters
        if (0 != at_check_digital_uint32_t(param->argv[0], &frequency))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[1], &spreading_factor))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[2], &bandwidth))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[3], &coding_rate))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[4], &preamble_length))
            return AT_PARAM_ERROR;
        if (0 != at_check_digital_uint32_t(param->argv[5], &txpower))
            return AT_PARAM_ERROR;
		if (0 != at_check_digital_uint32_t(param->argv[6], &low_data_rate_optimize))
            return AT_PARAM_ERROR;
		if (0 != at_check_digital_uint32_t(param->argv[7], &crc_on))
            return AT_PARAM_ERROR;
		if (0 != at_check_digital_uint32_t(param->argv[8], &rxgain))
            return AT_PARAM_ERROR;
		if (0 != at_check_digital_uint32_t(param->argv[9], &payload_len))
            return AT_PARAM_ERROR;	

        // Compatible old SPEC for bandwidth
        if (SERVICE_LORA_P2P == service_lora_p2p_get_nwm()) {
            if( bandwidth == 125 ) {
                bandwidth = 0;
            }
            else if( bandwidth == 250 ) {
                bandwidth = 1;
            }
            else if( bandwidth == 500 ) {
                bandwidth = 2;
            }
        }

        // Check parameters
		udrv_code = service_lora_p2p_check_runtime_freq(frequency);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P2_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_check_runtime_sf((uint8_t)spreading_factor);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P2_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_check_runtime_bandwidth(bandwidth);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P2_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_check_runtime_codingrate((uint8_t)coding_rate);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P2_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_check_runtime_preamlen((uint16_t)preamble_length);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P2_CHECK_ERROR_CODE;
        udrv_code = service_lora_p2p_check_runtime_powerdbm((uint8_t)txpower);
        if( udrv_code != UDRV_RETURN_OK)
            goto STEP_ATP2P2_CHECK_ERROR_CODE;

		if (payload_len > 255)
			return AT_PARAM_ERROR;

        runtimeConfigP2P.frequency = frequency;
        runtimeConfigP2P.spreading_factor = (uint8_t)spreading_factor;
        if (SERVICE_LORA_P2P == service_lora_p2p_get_nwm()) {
            runtimeConfigP2P.bandwidth = bandwidth;
        }
        else if (SERVICE_LORA_FSK == service_lora_p2p_get_nwm()) {
            runtimeConfigP2P.fsk_rxbw = bandwidth;
        }
        runtimeConfigP2P.coding_rate = (uint8_t)coding_rate;
        runtimeConfigP2P.preamble_length = (uint16_t)preamble_length;
        runtimeConfigP2P.txpower = (uint8_t)txpower;
		runtimeConfigP2P.low_data_rate_optimize = low_data_rate_optimize != 0;
		runtimeConfigP2P.crc_on = crc_on != 0;
        runtimeConfigP2P.rxgain = rxgain != 0;
		runtimeConfigP2P.payload_len = (uint8_t)payload_len;
		runtimeConfigP2P.fix_length_payload = payload_len > 0;

        // Can only configure when recieve mode not enabled
        api.lora.precv(0);

        set_runtimeConfigP2P(&runtimeConfigP2P);
        set_useRuntimeConfigP2P(true);
        service_lora_p2p_config();
        api.lora.precv(65533);
        return AT_OK;

        STEP_ATP2P2_CHECK_ERROR_CODE:
        //Restore the previous parameters
        if (!o_useRuntimeConfig) {
            service_lora_p2p_set_freq(o_frequency);
            service_lora_p2p_set_sf((uint8_t)o_spreading_factor);
            service_lora_p2p_set_bandwidth(o_bandwidth);
            service_lora_p2p_set_codingrate((uint8_t)o_coding_rate);
            service_lora_p2p_set_preamlen((uint16_t)o_preamble_length);
            service_lora_p2p_set_powerdbm((uint8_t)o_txpower);
			service_lora_p2p_set_low_datarate_optimize(o_low_data_rate_optimize);
			service_lora_p2p_set_crcon(o_crc_on);
            service_lora_p2p_set_rxgain(o_rxgain);
			service_lora_p2p_set_payloadlen(o_payload_len);
			service_lora_p2p_set_fix_length_payload(o_fix_length_payload);
        }
        api.lora.precv(65533);
        //Check and return error code
        return at_error_code_form_udrv(udrv_code);
    }
	else {
        return AT_PARAM_ERROR;
    }
}


// SEND //
void send_cb(void) {
	// Finished sending, turn off red led
    digitalWrite(LED_RED_TRANSMIT, HIGH);
}

void cad_cb(bool detect) {
	if(detect)
    {
        digitalWrite(LED_BLUE_RECEIVE, LOW);
		digitalWrite(LED_RED_TRANSMIT, LOW);
        delay(100); // todo: if keeping this then set timer to not delay response
        digitalWrite(LED_BLUE_RECEIVE, HIGH);
        digitalWrite(LED_RED_TRANSMIT, HIGH);
    }
	else
		digitalWrite(LED_RED_TRANSMIT, LOW);
    
}

/**
 * @brief Send data in P2P mode with the provided payload
 *
 * @param port Serial port used
 * @param cmd char array with the received AT command
 * @param param char array with the received AT command parameters
 * @return int result of command parsing
 * 			AT_OK AT command & parameters valid
 * 			AT_PARAM_ERROR command or parameters invalid
 */
int send_handler(SERIAL_PORT port, char *cmd, stParam *param)
{
    if (param->argc == 1 && !strcmp(param->argv[0], "?"))
    {
        return AT_PARAM_ERROR;
    }
    else if (param->argc == 1)
    {
        digitalWrite(LED_RED_TRANSMIT, LOW);
        uint32_t datalen;
        uint8_t lora_data[256];
        
        datalen = strlen(param->argv[0]);
        if (0 != at_check_hex_param(param->argv[0], datalen, lora_data))
            return AT_PARAM_ERROR;
        bool sentOK = api.lora.psend(datalen / 2, lora_data);
        if (sentOK) {
            return AT_OK;
        } else {
            return AT_BUSY_ERROR;
        }
    }
    else
    {
        return AT_PARAM_ERROR;
    }
}

// RECEIVE
void turn_off_receive_led(void *)
{
    digitalWrite(LED_BLUE_RECEIVE, HIGH);
}

void receive_cb(rui_lora_p2p_recv_t recv_data_pkg) {
    // Drop packets that are too large
    if (recv_data_pkg.BufferSize > sizeof(LoraMeessage_t::Buffer)) {
        return;
    }

    digitalWrite(LED_BLUE_RECEIVE, LOW);
    LoraMeessage_t theMessage;
    theMessage.BufferSize = recv_data_pkg.BufferSize;
    memcpy(theMessage.Buffer, recv_data_pkg.Buffer, recv_data_pkg.BufferSize);
    theMessage.Rssi = recv_data_pkg.Rssi;
    theMessage.Snr = recv_data_pkg.Snr;
    theMessage.Status = recv_data_pkg.Status;
	MessageQueue_enQueue(&incomingMessageQueue, &theMessage);

    api.system.timer.create(RAK_TIMER_0, turn_off_receive_led, RAK_TIMER_ONESHOT);
	// Trigger to turn of receive led 
	api.system.timer.start(RAK_TIMER_0, 500, NULL);
}

 

static void p2p_printf_hex(uint8_t *pdata, uint16_t len)
{       
    while(len--)
        udrv_serial_log_printf("%02X", *pdata++);
} 

/**
 * @brief Receive data in P2P mode
 *
 * @param port Serial port used
 * @param cmd char array with the received AT command
 * @param param char array with the received AT command parameters
 * @return int result of command parsing
 * 			AT_OK AT command & parameters valid
 * 			AT_PARAM_ERROR command or parameters invalid
 *          AT_EMPTY ( "EM" ) if no data to return
 *          AT_NO_STATUS ( "" ) when there is data -> no status message returned
 */
int receive_handler(SERIAL_PORT port, char *cmd, stParam *param)
{
    /*
    HardwareSerial* serialX;
    if (port == SERIAL_UART1)
    {
        serialX = &Serial1;
    }
    else if (port == SERIAL_UART2)
    {
        serialX = &Serial;
    }
    AT_PRINTF(serialX, "argc %d", param->argc);
    AT_PRINTF(serialX, "arcv %s", param->argv[0]);
    */
    if (param->argc == 0 
        ||
        (param->argc == 1 && !strcmp(param->argv[0], "?"))
        ||
        (param->argc == 1 && !strcmp(param->argv[0], "H"))
       )
    {
        bool hexFormat = false;
        if (param->argc == 1 && !strcmp(param->argv[0], "H")) {
            hexFormat = true;
        }
        if (MessageQueue_isEmpty(&incomingMessageQueue)) {
            return AT_EMPTY;
        } else {
            LoraMeessage_t msg;
            if (MessageQueue_deQueue(&incomingMessageQueue, &msg))
            {
                atcmd_printf("OK");
                if (hexFormat) {
                    atcmd_printf(":");
                    p2p_printf_hex(msg.Buffer, msg.BufferSize);
                    atcmd_printf(":%d:%d:%d", msg.Rssi, msg.Snr, msg.Status);
                    return AT_NO_STATUS;
                } else {
                    HardwareSerial* serialX;
                    if (port == SERIAL_UART1)
                    {
                        serialX = &Serial1;
                    }
                    else if (port == SERIAL_UART2)
                    {
                        serialX = &Serial;
                    }
                    for (uint8_t i=0; i<msg.BufferSize; i++)
                    {
                        serialX->write(msg.Buffer[i]);
                    }
                    uint8_t rssiHigh = (msg.Rssi >> 8) & 0xFF;
                    uint8_t rssiLow = msg.Rssi & 0xFF;
                    serialX->write(rssiHigh);
                    serialX->write(rssiLow);
                    serialX->write(msg.Snr);
                    serialX->write(msg.Status);
                    return AT_NO_STATUS;
                }
            }
        }
    }
    else
    {
        return AT_PARAM_ERROR;
    }
}