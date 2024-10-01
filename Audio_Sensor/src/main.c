/*sdk verison 2.5.1*/
#include "headers/main_aws.h"
#include "headers/main_audio.h"
#include "headers/main_gsm.h"
#include "headers/main_epoch.h"
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define SPI_FLASH_SECTOR_SIZE 0x1000
#define READ_ADDRESS 0x1000
#define WRITE_ADDRESS 0x2000

/* GPIO led code */
// Initialization of led1 to start GSM
#define LED0_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* Defining the mutex as to lock the flash while reading and writing and then unlocking it. */
K_MUTEX_DEFINE(flash_mutex);

int devId = 0x16;
int writeIdx = 0;
int readIdx = 0;
int maxWriteIdx = 300000; // for 32GB SD Card
int maxReadIdx = 300000;  // for 32GB SD Card
int flahsLimit = 0x800000;

void gsm_enable()
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
	gpio_pin_set_dt(&led, 0);
	K_MSEC(300);
	gpio_pin_set_dt(&led, 1);
	K_MSEC(100);
}

void main(void)
{
	printk("Device ID is : %x\n", devId);
	printk("Starting Board! %s\n", CONFIG_BOARD);
	int rc;
	int tempReadIdx, tempWriteIdx;
	const struct device *flash_dev;
	flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	size_t len = sizeof(int);
	if (!device_is_ready(flash_dev))
	{
		printk("%s: flash device device not ready.\n", flash_dev->name);
		return;
	}
	rc = flash_read(flash_dev, READ_ADDRESS, &tempReadIdx, len);
	if (rc == 0)
	{
		printk("Successfully read readIdx : %d\n", tempReadIdx);
		if (tempReadIdx > 0 && tempReadIdx < maxReadIdx)
		{
			readIdx = tempReadIdx;
		}
	}
	else
	{
		printk("Failed to read readIdx\n");
	}
	rc = flash_read(flash_dev, WRITE_ADDRESS, &tempWriteIdx, len);
	if (rc == 0)
	{
		printk("Successfully read writeIdx : %d\n", tempWriteIdx);
		if (tempWriteIdx > 0 && tempWriteIdx < maxWriteIdx)
		{
			writeIdx = tempWriteIdx;
		}
	}
	else
	{
		printk("Failed to read writeIdx\n");
	}
	Audio_init();
	gsm_enable();
	init_gsm(&GSM_CONNECTED, &GSM_DISCONNECTED);
	init_aws();
	AWS_loop();
}