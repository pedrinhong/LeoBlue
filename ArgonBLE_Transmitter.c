/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Peripheral Heart Rate over LE Coded PHY sample
 */
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/hrs.h>

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/drivers/i2c.h>

//#include <Wire.h>

static struct bt_le_ext_adv *adv;

#define K_SECONDS(s) K_MSEC((s) * MSEC_PER_SEC)
#define K_WORK_DEFINE(work,work_handler) struct k_work work = Z_WORK_INITIALIZER(work_handler)
#define K_TIMER_DEFINE(name,expiry_fn,stop_fn) STRUCT_SECTION_ITERABLE(k_timer, name) = Z_TIMER_INITIALIZER(name, expiry_fn, stop_fn)

#define DEVICE_NAME            "PEDRO_NOG"
#define DEVICE_NAME_LEN        (sizeof(DEVICE_NAME) - 1)

#define COMPANY_ID_CODE            0x0059

// IO constants
#define RUN_STATUS_LED          DK_LED1
#define CON_STATUS_LED          DK_LED2
#define RUN_LED_BLINK_INTERVAL  1000
#define VCTL1_PIN 15
#define VCTL2_PIN 16
#define VCTL3_PIN 14
#define CTLANT_002 2
#define CTLANT_025 25
#define USER_BUTTON DK_BTN1_MSK

// I2C constants
// #define I2C_ADDRESS 0x08
// #define I2C_DEV 
//#define I2C_NODE DT_NODELABEL(mysla)
#define I2C_NODE DT_PATH(soc, i2c_40003000, mysla_4a)
#define GPIO_NODE DT_ALIAS(gpiocus0)
// #define GPIO_NODE025 DT_ALIAS(gpiocus025)
// #define GPIO_NODE002 DT_ALIAS(gpiocus002)
#define GPIO_PIN 4

//Eirballon --> //,0x10,0x22,0x45, 0x49, 0x52, 0x42, 0x41, 0x4C, 0x4C, 0x4F, 0x4F,0x4E, 0x34};

//BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN
//uint8_t DATA[] = {0x4C,0x65,0x6F,0x42,0x6C,0x75,0x65};
const struct device *dev_gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
//static struct hrs_cb hrs_cb;
uint8_t DATA_2[]={0x54,0x18,0x00,0x5A,0xFF,0x46,0x49,0x52,0x45};
bool advOn = true, firstOn = true, firstOff = false;
int count=0;

const long long TRAME0 = 0x0000000000000000;
const long long TRAME1 = 0xAAAAAAAAAAAAAAAA;
const long long TRAME2 = 0xBBBBBBBBBBBBBBBB;
const long long TRAME3 = 0xCCCCCCCCCCCCCCCC;

long long packet0[20] = {TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0};
long long packet1[20] = {TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1,TRAME1};
long long packet2[20] = {TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2,TRAME2};
long long packet3[20] = {TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3,TRAME3};

// -- DATA STRUCTURES

typedef struct adv_mfg_data {
	//uint16_t company_code;	    /* Company Identifier Code. */
	long long number_press[20];      /* Number of times Button 1 is pressed*/
} adv_mfg_data_type;
//,TRAME,TRAME,TRAME,TRAME,TRAME,TRAME,TRAME,TRAME,TRAME,TRAME,TRAME,TRAME
static adv_mfg_data_type adv_mfg_data = {{TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0,TRAME0}};//{COMPANY_ID_CODE,TRAME};
//char* device_name = "PEDRONOG";


// WHAT IS GOING TO BE SHOWN DURING ADVERTISING.
static const struct bt_data ad[] = {
	// --- SETTING ADVERTISING FLAGS
	//BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	// Both flags are used for a device in peripheral role.
	// BT_LE_AD_GENERAL Sets LE General Discoverable Mode, used in connectable advertising
	// to indicate that advertising is available for a long period of time
	// BT_LE_AD_NO_BREDR states that classical Bluetooth is not supported.

	// --- SETTING ADVERTISING PACKET DATA USING ADVERTISIMENT DATA STRUCTURES.
	// Complete name
	//BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	// Service UUID16
	//BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)),
	//BT_DATA(0x0809,DATA,sizeof(DATA)),
	// Service data
	//BT_DATA(BT_DATA_SVC_DATA16, DATA_2,sizeof(DATA_2)),
	//BT_DATA(0x0A16,DATA_2,sizeof(DATA_2))
	// Manufacturer Specific Data
	BT_DATA(BT_DATA_MANUFACTURER_DATA,(unsigned char *)&adv_mfg_data, sizeof(adv_mfg_data)),
};

// Variables for I2C.
static struct gpio_callback pin_cb_data;
static volatile bool read = false;
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);
static const struct gpio_dt_spec request_pin = GPIO_DT_SPEC_GET(GPIO_NODE, gpios);


// BLE BUtton Callback function
// Enable or disable the advertising by button click
// static void button_changed(uint32_t button_state, uint32_t has_changed){
// 	int err;
// 	if (has_changed & button_state & USER_BUTTON) {
// 		advOn = !advOn;
// 		if(advOn){
// 			firstOn = true;
// 			gpio_pin_set(dev_gpio0, VCTL2_PIN, 1);
// 		}
// 		else{
// 			firstOff = true;
// 			gpio_pin_set(dev_gpio0, VCTL2_PIN, 0);
// 		}
// 	}
// }

// CALLED BY ADV_UPDATE
static int create_advertising_coded(int mix){
	int err;
	// BT_LE_ADV_OPT_CONNECTABLE |
	// Determining some aspects of the advertising. Such as advertising interval and kind of modulation.
	// BT_LE_ADV_PARAM_INIT(options, adv_interval_min, adv_interval_max, peer)

	// UNCODED 1M
	struct bt_le_adv_param param_1 =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_EXT_ADV |
				     BT_LE_ADV_OPT_NO_2M |BT_LE_ADV_OPT_USE_TX_POWER,
				     BT_GAP_ADV_FAST_INT_MIN_2,
					 BT_GAP_ADV_FAST_INT_MAX_2,
				     NULL);

	// CODED 125k
	struct bt_le_adv_param param_2 =
		BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_EXT_ADV |
				     BT_LE_ADV_OPT_CODED |BT_LE_ADV_OPT_USE_TX_POWER,
				     BT_GAP_ADV_FAST_INT_MIN_2,
					 BT_GAP_ADV_FAST_INT_MAX_2,
				     NULL);

	struct bt_le_adv_param param;
		
	if(mix == 0){
		param = param_1;
	}
	else{
		param = param_2;
	}

	err = bt_le_ext_adv_create(&param, NULL, &adv);
	if (err) {
		printk("Failed to create advertiser set (err %d)\n", err);
		return err;
	}

	printk("Created adv: %p\n", adv);

	// AD -> ADV
	err = bt_le_ext_adv_set_data(adv, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Failed to set advertising data (err %d)\n", err);
		return err;
	}
	
	return 0;
}

// ENABLING AND STARTING EXTENDED ADVERTISING.
int adv_start(int mix){
	int err;
	err = bt_enable(NULL); // Enabling Bluetooth stack.
	
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return err;
	}

	printk("Bluetooth initialized\n");
	err = create_advertising_coded(mix);
	if (err) {
		printk("Advertising failed to create (err %d)\n", err);
		return err;
	}

	err = bt_le_ext_adv_start(adv, NULL); // Starting extended advertising.
	if (err) {
		printk("Advertising failed to send data (err %d)\n", err);
		return err;
	}

	return 0;
}

// Used for updating data.
int adv_break(void){
	int err;
	err= bt_le_ext_adv_stop(adv);
	if (err) {
		printk("Advertising failed to stop (err %d)\n", err);
		return err;
	}
	err = bt_disable();
	if (!err) {
		printk("Stop BLE module \n");
		//return 0;
	}

	return 0;
}

void updateAdv(){
	if(advOn){
		// Stop advertising.
		if(!firstOn){
			adv_break();
		}
		// Then resetting data and restarting advertising.
		adv_start(count%2);
		firstOn = false;
	}
	else{
		if(firstOff){
			adv_break();
			firstOff = false;
		}
	}
	return;
}

void my_work_handler(struct k_work *work){
	count += 1;
	gpio_pin_set(dev_gpio0, VCTL3_PIN, count%2);
	updateAdv();
}

K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&my_work);
}

K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

// // Button initialisation
// static int init_button(void){
// 	int err;
// 	err = dk_buttons_init(button_changed);
// 	if (err) {
// 		printk("Cannot init buttons (err: %d)\n", err);
// 	}
// 	return err;
// }

int init_gpio(){
	// - start
	if (!device_is_ready(dev_gpio0)) {
		return -1;
	}
	// configure the LED pin as output
	gpio_pin_configure(dev_gpio0, VCTL1_PIN, GPIO_OUTPUT_INACTIVE);
  	gpio_pin_configure(dev_gpio0, VCTL2_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(dev_gpio0, VCTL3_PIN, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(dev_gpio0, CTLANT_002, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(dev_gpio0, CTLANT_025, GPIO_OUTPUT_INACTIVE);
	

	// LED 3 off and LED 4 on
	gpio_pin_set(dev_gpio0, VCTL1_PIN, 1);
	gpio_pin_set(dev_gpio0, VCTL2_PIN, 0);
	// gpio_pin_set(dev_gpio0, CTLANT_002, 0);
	// gpio_pin_set(dev_gpio0, CTLANT_025, 1);

	return 0;
}


// Functions for I2C link: button interruption and init functions.
void pin_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Data is available to be read.\n");
	read = true;
	// uint8_t data[2];
	// int ret = i2c_read_dt(&dev_i2c, &data, sizeof(data));
	// if(ret != 0){
	// 	printk("Failed to read from I2C device address %x \n.", dev_i2c.addr);
	// }
	// //k_msleep(SLEEP_TIME_MS);
	// printk("Read data = %d and %d.\n", data[0], data[1]);
	// read = false;
}

int i2c_init(){
	/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is ready to use  */
	if (!device_is_ready(dev_i2c.bus)) {
		return -1;
	}
	printk("I2C Initialized.\n");
	return 0;
}

int gpio_i2c_request_button_init(){
	
	if (!gpio_is_ready_dt(&request_pin)) {
		return -1;
	}

	gpio_pin_configure_dt(&request_pin, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&request_pin,GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&pin_cb_data, pin_isr, BIT(request_pin.pin));
	gpio_add_callback(request_pin.port, &pin_cb_data);

	printk("GPIO for I2C request button Initialized \n.");
	return 0;
}

int main(void){
	uint32_t led_status = 0;
	int err;
	int val=1;

	printk("Starting Bluetooth Peripheral HR coded example\n");
	// get the GPIO device
	//const struct device *dev_gpio0;

	// GPIO init.
    err = init_gpio();
	if(err == -1){
		printk("main: gpio not ready\n");
	}
	// Buttons init.
	// err = init_button();
	// if (err) {
	// 	printk("Button init failed (err %d)\n", err);
	// 	return -1;
	// }
	// Leds init.
	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return err;
	}
	// I2C init.
	err = i2c_init();
	if(err == -1){
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
	}
	// Reading request button init.
	err = gpio_i2c_request_button_init();
	if(err == -1){
		printk("GPIO for I2C request button is not ready!");
	}


	/* start a periodic timer that expires once every second */
	k_timer_start(&my_timer, K_SECONDS(3), K_SECONDS(3));
	//gpio_pin_set(dev_gpio0, VCTL3_PIN, 1);
	//dk_set_led(RUN_STATUS_LED, 1);
	//adv_start(0);

	while(1) {
		dk_set_led(RUN_STATUS_LED, (++led_status) % 2);
		//gpio_pin_set(dev_gpio0, VCTL3_PIN, led_status%2);
		if(read){
			uint8_t data[2];
			err = i2c_read_dt(&dev_i2c, &data, sizeof(data));
			if(err != 0){
				printk("Failed to read from I2C device address %x \n.", dev_i2c.addr);
			}
			if(data[0] == 0 && data[1] == 0){
				memcpy(adv_mfg_data.number_press, packet1, sizeof(packet1));
				updateAdv();
				printk("A disaster has been detected. Disaster type: Tsunami. \n");
			}
			else if(data[0] == 1 && data[1] == 1){
				memcpy(adv_mfg_data.number_press, packet2, sizeof(packet2));
				updateAdv();
				printk("A disaster has been detected. Disaster type: Fire.\n");
			}
			else if(data[0] == 2 && data[1] == 2){
				memcpy(adv_mfg_data.number_press, packet3, sizeof(packet3));
				updateAdv();
				printk("A disaster has been detected. Disaster type: Earthquake.\n");
			}
			else{
				memcpy(adv_mfg_data.number_press, packet0, sizeof(packet0));
				updateAdv();
				printk("A disaster has been detected. Disaster type: Not identified.\n");
			}
			//printk("Read data = %d and %d.\n", data[0], data[1]);
			read = false;
		}
		k_sleep(K_MSEC(RUN_LED_BLINK_INTERVAL));
	}
}