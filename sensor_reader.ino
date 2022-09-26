/*
 Name:		foundation_moisture.ino
 Created:	9/2/2022 8:51:50 AM
 Author:	user
*/

//---------------------------------------------------------------------------------------------------------------
// Sensors in use
//---------------------------------------------------------------------------------------------------------------
#define HTU_SENSOR_ACTIVE x
// #define DHT_SENSOR_ACTIVE x
// #define BMP_SENSOR_ACTIVE x
// #define VBAT_SENSOR_ACTIVE x

//---------------------------------------------------------------------------------------------------------------
// Libraries
//---------------------------------------------------------------------------------------------------------------
#include <modigdesign_oled.h>
#include <modigdesign_ttn.h>
#include <modigdesign_debug.h>
#include <modigdesign_funcs.h>
#include <modigdesign_ota.h>
#include <modigdesign_wifi.h>
#include <modigdesign_secrets.h>
#include <modigdesign_bmp.h>
#include <modigdesign_vbat.h>
#include <modigdesign_dht.h>
#include <modigdesign_htu21d.h>

//---------------------------------------------------------------------------------------------------------------
// Heltec WiFi LoRa 32 V2 - GPIO Usage
//---------------------------------------------------------------------------------------------------------------
static const gpio_num_t GPIO_SDA_2 = GPIO_NUM_23;  // OLED SDA is on GPIO 4
static const gpio_num_t GPIO_SCL_2 = GPIO_NUM_22;  // OLED SCL is on GPIO 15
static const gpio_num_t GPIO_DHT = GPIO_NUM_17;
static const gpio_num_t GPIO_PRG_BUTTON = GPIO_NUM_0;
static const gpio_num_t GPIO_LED = GPIO_NUM_25;
static const gpio_num_t GPIO_VOLTAGE_SENSOR = GPIO_NUM_37;
static const gpio_num_t GPIO_VEXT = GPIO_NUM_21;

//---------------------------------------------------------------------------------------------------------------
// Constants
//---------------------------------------------------------------------------------------------------------------
static const uint8_t READ_INTERVAL_S = 20;		// Interval for sensors to do five measurements
static const uint8_t LOW_BATTERY_LIMIT = 20;	// Battery % below this value triggers LOW_BATTERY error 
static const float MAX_BATTERY_VOLTAGE = 4.2;	// Installed battery fully charged voltage
static const float MIN_BATTERY_VOLTAGE = 2.5;	// Voltage when battery is considered empty
static const uint64_t SLEEP_TIME_S = 900;		// Time between measure activity
static const uint8_t SLEEP_TIMEOUT_S = 60;		// Goto sleep after...
static const uint8_t SENSOR_TIMEOUT_S = 120;	// Send error and goto sleep when sensors haven't responded after...

//---------------------------------------------------------------------------------------------------------------
// Debug
//---------------------------------------------------------------------------------------------------------------
static const uint32_t SERIAL_BAUDRATE = 115200;	
static const modigdesign_debug_class::debug_levels_t DEBUG_LEVEL = modigdesign_debug_class::LEVEL_VERBOSE;

//---------------------------------------------------------------------------------------------------------------
// Running mode
//---------------------------------------------------------------------------------------------------------------
enum running_mode_t {
	TIMER_MODE,
	MANUAL_MODE,
	RESET_MODE,
	OTA_MODE
};
running_mode_t running_mode;
String running_mode_text(running_mode_t mode) {
	switch (mode) {
	case TIMER_MODE:	return "TIMER_MODE";
	case RESET_MODE:	return "RESET_MODE";
	case MANUAL_MODE:   return "MANUAL_MODE";
	case OTA_MODE:		return "OTA_MODE";
	default:			return "Unknown";
	}
}

//---------------------------------------------------------------------------------------------------------------
// Public variables
//---------------------------------------------------------------------------------------------------------------
RTC_DATA_ATTR int boot_count = 0;
volatile bool goto_sleep_pending = false;
modigdesign_secrets_class::board_data_t board_data;
modigdesign_funcs_class::led_class led(GPIO_LED);

float dht_temperature;
float dht_humidity;
float htu_temperature;
float htu_humidity;
float battery_voltage;
int battery_status;
float bmp_temperature;
float bmp_pressure;


//---------------------------------------------------------------------------------------------------------------
// Set running mode
//---------------------------------------------------------------------------------------------------------------
void set_running_mode() {

	esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
	uint64_t status = esp_sleep_get_ext1_wakeup_status();
	switch (wakeup_reason)
	{
	case ESP_SLEEP_WAKEUP_EXT0: 
		DEBUG_I("ESP_SLEEP_WAKEUP_EXT0");
		running_mode = MANUAL_MODE;
		break;
	case ESP_SLEEP_WAKEUP_EXT1:
		DEBUG_I("ESP_SLEEP_WAKEUP_EXT1");
		running_mode = MANUAL_MODE;
		break;
	case ESP_SLEEP_WAKEUP_TIMER:
		DEBUG_I("ESP_SLEEP_WAKEUP_TIMER"); 
		running_mode = TIMER_MODE;
		break;
	case ESP_SLEEP_WAKEUP_TOUCHPAD:
		DEBUG_I("ESP_SLEEP_WAKEUP_TOUCHPAD");
		running_mode = MANUAL_MODE;
		break;
	case ESP_SLEEP_WAKEUP_ULP:
		DEBUG_I("ESP_SLEEP_WAKEUP_ULP");
		running_mode = MANUAL_MODE;
		break;
	default:
		running_mode = RESET_MODE;
		break;
	}

	String startup_text = running_mode_text(running_mode);
	DEBUG_I("Startup caused by %s", startup_text);

}

//---------------------------------------------------------------------------------------------------------------
// Goto sleep
//---------------------------------------------------------------------------------------------------------------
void goto_sleep(bool permanent = false) {
	DEBUG_I("Going to sleep...");
	digitalWrite(GPIO_VEXT, HIGH);
	if (!permanent)
		esp_sleep_enable_timer_wakeup(SLEEP_TIME_S * 1000000);
	esp_deep_sleep_start();
}

//---------------------------------------------------------------------------------------------------------------
// Send read values via TTN
//---------------------------------------------------------------------------------------------------------------
void send_values() {

	// Message
	DEBUG_V("Sending values...");
	if (running_mode != TIMER_MODE) {
		DEBUG_V("OLED...");
		// OLED->alert("LoRa message...");
	}

	// Prepare TTN message
	DEBUG_V("New TTN message...");
	TTN->clear_message();

	// DHT values
#ifdef DHT_SENSOR_ACTIVE
	dht_temperature = DHT->get_temperature_value();
	dht_humidity = DHT->get_humidity_value();
	DEBUG_I("Sending DHT values Temperature: %.1f (C) Humidity: %.0f (%%)", dht_temperature, dht_humidity);
	TTN->add_temperature(dht_temperature)
		.add_relative_humidity(dht_humidity);
	DHT->reset_readings();
#endif

	// HTU21D values
#ifdef HTU_SENSOR_ACTIVE
	htu_temperature = HTU21D->get_temperature_value();
	htu_humidity = HTU21D->get_humidity_value();
	DEBUG_I("Sending HTU21D values Temperature: %.1f (C) Humidity: %.0f (%%)", htu_temperature, htu_humidity);
	TTN->add_temperature(htu_temperature)
		.add_relative_humidity(htu_humidity);
	HTU21D->reset_readings();
#endif

	// BMP values
#ifdef BMP_SENSOR_ACTIVE
	bmp_temperature = BMP->get_temperature_value();
	bmp_pressure = BMP->get_pressure_value();
	DEBUG_I("Sending BMP values Temperature: %.1f (C) Pressure: %.0f (%%)", bmp_temperature, bmp_pressure);
	TTN->add_barometric_pressure(bmp_pressure/100);
	BMP->reset_readings();
#endif

	// Battery values
#ifdef VBAT_SENSOR_ACTIVE
	battery_voltage = VBAT->get_voltage();
	battery_status = VBAT->get_percentage();
	DEBUG_I("Sending VBAT values Voltage: %.1f (V) Status: %d (%%)", battery_voltage, battery_status);
	TTN->add_voltage(battery_voltage)
		.add_battery(battery_status)
		.add_error(battery_status > LOW_BATTERY_LIMIT ? modigdesign_ttn_class::OK : modigdesign_ttn_class::LOW_BATTERY);
	VBAT->reset_readings();
#endif 

	// Send the message
	TTN->send_message();

}

//---------------------------------------------------------------------------------------------------------------
// Display on OLED
//---------------------------------------------------------------------------------------------------------------
void display_on_oled() {

	static int counter = 0;
	DEBUG_V("Displaying result on OLED...");

	switch (counter % 3) {
#ifdef DHT_SENSOR_ACTIVE
	case 0:
		OLED->clear_screen()
			.set_font_size(OLEDDISPLAY_FONT_SIZE::MEDIUM)
			.set_text_alignment(OLEDDISPLAY_TEXT_ALIGNMENT::TEXT_ALIGN_CENTER)
			.print(1, "DHT Values")
			.set_font_size(OLEDDISPLAY_FONT_SIZE::SMALL)
			.printf_cols(3, "Temperature:", "%.1f (C)", dht_temperature)
			.printf_cols(4, "Humidity:", "%.0f (%%)", dht_humidity);
		break;
#endif
#ifdef BMP_SENSOR_ACTIVE
	case 1:
		OLED->clear_screen()
			.set_font_size(OLEDDISPLAY_FONT_SIZE::MEDIUM)
			.set_text_alignment(OLEDDISPLAY_TEXT_ALIGNMENT::TEXT_ALIGN_CENTER)
			.print(1, "BMP Values")
			.set_font_size(OLEDDISPLAY_FONT_SIZE::SMALL)
			.printf_cols(3, "Temperature:", "%.1f (C)", bmp_temperature)
			.printf_cols(4, "Pressure:", "%.0f (hPa)", bmp_pressure/100);
		break;
#endif
#ifdef HTU_SENSOR_ACTIVE
	case 2:
		OLED->clear_screen()
			.set_font_size(OLEDDISPLAY_FONT_SIZE::MEDIUM)
			.set_text_alignment(OLEDDISPLAY_TEXT_ALIGNMENT::TEXT_ALIGN_CENTER)
			.print(1, "HTU21D Values")
			.set_font_size(OLEDDISPLAY_FONT_SIZE::SMALL)
			.printf_cols(3, "Temperature:", "%.1f (C)", htu_temperature)
			.printf_cols(4, "Humidity:", "%.0f (%%)", htu_humidity);
		break;
#endif
#ifdef VBAT_SENSOR_ACTIVE
	case 3:
		OLED->clear_screen()
			.set_font_size(OLEDDISPLAY_FONT_SIZE::MEDIUM)
			.set_text_alignment(OLEDDISPLAY_TEXT_ALIGNMENT::TEXT_ALIGN_CENTER)
			.print(1, "Battery status")
			.set_font_size(OLEDDISPLAY_FONT_SIZE::SMALL)
			.printf_cols(3, "Voltage:", "%.1f (V)", battery_voltage)
			.printf_cols(4, "Status:", "%d (%%)", battery_status);
		break;
#endif
	}
	counter++;

}

//---------------------------------------------------------------------------------------------------------------
// Start OLED
//---------------------------------------------------------------------------------------------------------------
void start_oled() {
	static bool started;
	if (!started) {
		OLED->set_scl_pin(15)
			.set_sda_pin(4)
			.set_reset_pin(16)
			.set_i2c_address(0x3c)
			.set_orientation(OLEDDISPLAY_ORIENTATION::LANDSCAPE_REV)
			.begin()
			.alert("Waiting for sensor(s)...");
		started = true;
	}
}

//---------------------------------------------------------------------------------------------------------------
// Start OTA mode
//---------------------------------------------------------------------------------------------------------------
void prg_button_pressed() {
	running_mode = OTA_MODE;
}
void start_ota() {

	DEBUG_I("Starting OTA...");

	// OLED
	start_oled();

	// WiFi
	modigdesign_wifi->set_wifi_ssid(modigdesign_secrets->get_wifi_ssid())
		.set_wifi_password(modigdesign_secrets->get_wifi_password())
		.wifi_begin();

	// OTA
	modigdesign_ota->set_host_name(board_data.name)
		.begin();

	// Display
	DEBUG_I("Waiting for update...");
	OLED->clear_screen()
		.alert("Waiting for update...");

	// Wait for update
	modigdesign_funcs_class::after_class after;
	while (true) {
		led.blink();
		if (after.minutes(5))
			break;
		modigdesign_ota->loop();
	}

}

//---------------------------------------------------------------------------------------------------------------
// Setup
//---------------------------------------------------------------------------------------------------------------
void setup() {

	// Boot counter
	boot_count++;

	// Logging
	Serial.begin(SERIAL_BAUDRATE);
	DEBUG->set_debug_level(DEBUG_LEVEL)
		.begin();

	// Determine running mode
	set_running_mode();

	// Start OLED?
	if (running_mode != TIMER_MODE)
		start_oled();

	// Pins
	pinMode(GPIO_DHT, INPUT);
	pinMode(GPIO_PRG_BUTTON, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(GPIO_PRG_BUTTON), prg_button_pressed, FALLING);

	// I2C (First I2C used by OLED)
	Wire1.begin(GPIO_SDA_2, GPIO_SCL_2);

	// Passwords and board data
	SECRETS->begin();
	WiFi.mode(WIFI_STA);
	board_data = modigdesign_secrets->get_board_data(WiFi.macAddress());

	// Start TTN
	TTN->set_frame_counter(800).begin(board_data.dev_addr, board_data.nwk_skey, board_data.app_skey);

	// DHT Sensor
#ifdef DHT_SENSOR_ACTIVE
	DHT->set_dht_type(DHT22)
		.set_read_pin(GPIO_DHT)
		.set_read_interval(READ_INTERVAL_S)
		.set_default_temperature_unit(DHT->Celsius)
		.begin();
#endif

	// DHT Sensor
#ifdef HTU_SENSOR_ACTIVE
	HTU21D->set_read_interval(READ_INTERVAL_S)
		.begin(&Wire1);
#endif

#ifdef BMP_SENSOR_ACTIVE
	BMP->set_i2c_address(0x76)
		.set_read_interval(READ_INTERVAL_S)
		.begin(&Wire1);
#endif

#ifdef VBAT_SENSOR_ACTIVE
	VBAT->set_max_voltage(MAX_BATTERY_VOLTAGE)
		.set_min_voltage(MIN_BATTERY_VOLTAGE)
		.set_read_pin(GPIO_VOLTAGE_SENSOR)
		.set_vext_pin(GPIO_VEXT)
		.set_samples(20)
		.set_read_interval(READ_INTERVAL_S)
		.begin();
	delay(2000);
	while (!VBAT->is_readings_ready())
		VBAT->loop();
	VBAT->reset_readings();
#else
	pinMode(GPIO_VEXT, OUTPUT);
	digitalWrite(GPIO_VEXT, LOW);
#endif

	// Wait for things to settle
	DEBUG_I("Setup done!");
	delay(100);

}

//---------------------------------------------------------------------------------------------------------------
// Background measurements
//---------------------------------------------------------------------------------------------------------------
bool background_measurements() {

	bool bmp = true;
	bool dht = true;
	bool vbat = true;
	bool htu = true;

#ifdef BMP_SENSOR_ACTIVE
	BMP->loop();
	bmp = BMP->is_readings_ready();
#endif
#ifdef HTU_SENSOR_ACTIVE
	HTU21D->loop();
	htu = HTU21D->is_readings_ready();
#endif
#ifdef DHT_SENSOR_ACTIVE
	DHT->loop();
	dht = DHT->is_readings_ready();
#endif
#ifdef VBAT_SENSOR_ACTIVE
	VBAT->loop();
	vbat = VBAT->is_readings_ready();
#endif

	if (bmp && dht && vbat && htu) {
		DEBUG_I("Readings finished!");
		return true;
	}
	return false;

}

//---------------------------------------------------------------------------------------------------------------
// Main loop
//---------------------------------------------------------------------------------------------------------------
int reading_count = 0;
modigdesign_funcs_class::every_class every;
modigdesign_funcs_class::after_class after;
void loop() {

	// When ready, send via TTN. Only one measurement is done.
	if (reading_count == 0 && background_measurements()) {
		reading_count++;
		DEBUG_I("Readings: %d", reading_count);
		send_values();
	}

	// Go to OTA mode
	if (running_mode == OTA_MODE)
		start_ota();

	// Go to sleep?
	if ((running_mode == TIMER_MODE && reading_count > 0))
		goto_sleep();
	if (goto_sleep_pending)
		goto_sleep();
	if (after.interval(SLEEP_TIMEOUT_S) && reading_count > 0)
		goto_sleep();

	// No value after x minutes => error
	if (after.interval(SENSOR_TIMEOUT_S) && reading_count == 0) {
		TTN->clear_message()
			.add_error(modigdesign_ttn_class::SENSOR_NOT_RESPONDING)
			.send_message();
		goto_sleep();
	}

	// Display result
	if (reading_count > 0 && every.interval(5)) {
		start_oled();
		display_on_oled();
	}

}
