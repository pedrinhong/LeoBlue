/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/gpio.h>
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */
// #define STTS751_TEMP_HIGH_REG            0x00
// #define STTS751_TEMP_LOW_REG             0x02
// #define STTS751_CONFIG_REG               0x03

/* STEP 6 - Get the node identifier of the sensor */
#define I2C_NODE DT_NODELABEL(mysla)
#define GPIO_NODE DT_ALIAS(gpiocus0)

#define GPIO_PIN 4

static struct gpio_callback pin_cb_data;
static volatile bool read = false;
static volatile const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

void pin_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Data is available to be read.\n");
	//read = true;
	uint8_t data[2];
	int ret = i2c_read_dt(&dev_i2c, &data, sizeof(data));
	if(ret != 0){
		printk("Failed to read from I2C device address %x \n.", dev_i2c.addr);
	}
	//k_msleep(SLEEP_TIME_MS);
	printk("Read data = %d and %d.\n", data[0], data[1]);
	read = false;
}

int main(void){

	int ret;

/* STEP 7 - Retrieve the API-specific device structure and make sure that the device is ready to use  */
	if (!device_is_ready(dev_i2c.bus)) {
		printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
		return -1;
	}
	printk("I2C Initialized.\n");


	static const struct gpio_dt_spec request_pin = GPIO_DT_SPEC_GET(GPIO_NODE, gpios);
	if (!gpio_is_ready_dt(&request_pin)) {
		return 0;
	}

	gpio_pin_configure_dt(&request_pin, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&request_pin,GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&pin_cb_data, pin_isr, BIT(request_pin.pin));
	gpio_add_callback(request_pin.port, &pin_cb_data);

	printk("GPIO Initialized \n.");

	while (1) {
		// if(read){
		// 	uint8_t data[2];
		// 	ret = i2c_read_dt(&dev_i2c, &data, sizeof(data));
		// 	if(ret != 0){
		// 		printk("Failed to read from I2C device address %x \n.", dev_i2c.addr);
		// 	}
		// 	//k_msleep(SLEEP_TIME_MS);
		// 	printk("Read data = %d and %d.\n", data[0], data[1]);
		// 	read = false;
		// }
	}
}
