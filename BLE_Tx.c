/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief Central Heart Rate over LE Coded PHY sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/gatt_dm.h>
#include <bluetooth/scan.h>
#include <bluetooth/services/hrs_client.h>
#include <dk_buttons_and_leds.h>

// MAC address of LeoBlue module 
#define FILTER_MAC_ADDR "FE:1F:73:A0:C8:4B" // Coded Tx
#define FILTER_MAC_ADDR_2 "E3:59:EB:87:54:72" // 1M Mbed based code
#define FILTER_MAC_ADDR_3 "DB:B6:9B:3C:88:ED" // Coded Tx
#define FILTER_MAC_ADDR_4 "F5:45:39:90:13:70"
#define FILTER_MAC_ADDR_5 "FB:C2:D4:59:E1:F5"

#define VCTL1_PIN 15

const struct device *dev_gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
bool state = true;

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *ad){

	// Convert received MAC @ to string value
	char addr_up[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(addr, addr_up, 18);

    // Check if the detected device's address matches the filter
    if ((strcmp(addr_up, FILTER_MAC_ADDR)==0)||(strcmp(addr_up, FILTER_MAC_ADDR_2)==0)||(strcmp(addr_up, FILTER_MAC_ADDR_3)==0)||(strcmp(addr_up, FILTER_MAC_ADDR_4)==0)||(strcmp(addr_up, FILTER_MAC_ADDR_5)==0))  {
		if(state){
			gpio_pin_set(dev_gpio0, VCTL1_PIN, 0);
			state = false;
		}
		else{
			gpio_pin_set(dev_gpio0, VCTL1_PIN, 1);
			state = true;
		}
        // Print only if the address matches the filter
        printk("Filtered Device - BD Address: %s, RSSI: %d dBm\n", addr_up, rssi);

        // Print the payload of the advertisement
        printk("Advertisement Data: ");
        for (int i = 0; i < ad->len*4; i++) {
            printk("%02X ", net_buf_simple_pull_u8(ad));
		}
		printk("\n --------------------------------\n");
	}
    
}


int main(void)
{
	int err;

	gpio_pin_configure(dev_gpio0, VCTL1_PIN, GPIO_OUTPUT_INACTIVE);

	printk("Starting Bluetooth Central HR coded example\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Début du scan BLE\n");

	struct bt_le_scan_param scan_param = {
		.type     = BT_LE_SCAN_TYPE_ACTIVE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window   = BT_GAP_SCAN_FAST_WINDOW,
		.options  = BT_LE_SCAN_OPT_CODED 
	};

	struct bt_scan_init_param scan_init = {
		.connect_if_match = 0,
		.scan_param = &scan_param,
		.conn_param = NULL
	};

	// Leds configuration
	err = dk_leds_init();
	if (err) {
		printk("LEDs init failed (err %d)\n", err);
		return err;
	}

	gpio_pin_set(dev_gpio0, VCTL1_PIN, 1);

	bt_scan_init(&scan_init);
	//bt_scan_cb_register(&scan_cb);
	err = bt_le_scan_start(&scan_param, scan_cb);
	//err = bt_scan_start(scan_cb);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return 0;
	}

	while(1);

	//bt_scan_stop();
	//printk("Fin du scan BLE\n");
	return 0;
}
