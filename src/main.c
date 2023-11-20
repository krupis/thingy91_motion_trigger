#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

K_SEM_DEFINE(sem, 0, 1);

static const enum sensor_channel channels[] = {
	SENSOR_CHAN_ACCEL_X,
	SENSOR_CHAN_ACCEL_Y,
	SENSOR_CHAN_ACCEL_Z,
};

#define DEFAULT_ADXL362_NODE DT_ALIAS(adxl362)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_ADXL362_NODE, okay),
			 "ADXL362 not specified in DT");

// DEVICE TREE STRUCTURE
const struct device *adxl1362_sens = DEVICE_DT_GET(DEFAULT_ADXL362_NODE);

static void trigger_handler(const struct device *dev,
							const struct sensor_trigger *trig)
{
	switch (trig->type)
	{
	case SENSOR_TRIG_DATA_READY:
	{
		printk("SENSOR_TRIG_DATA_READY\n");
		if (sensor_sample_fetch(dev) < 0)
		{
			printk("Sample fetch error\n");
			return;
		}
		k_sem_give(&sem);
		break;
	}
	case SENSOR_TRIG_MOTION:
	{
		printk("SENSOR_TRIG_MOTION\n");
		break;
	}

	case SENSOR_TRIG_STATIONARY:
	{
		printk("SENSOR_TRIG_STATIONARY\n");
		break;
	}

	default:
		printk("Unknown trigger\n");
	}
}

void main(void)
{

	struct sensor_value accel[3];

	if (!device_is_ready(adxl1362_sens))
	{
		printk("sensor: device %s not ready.\n", adxl1362_sens->name);
		return 0;
	}
	else
	{
		printk("sensor: device %s ready.\n", adxl1362_sens->name);
	}

	if (IS_ENABLED(CONFIG_ADXL362_TRIGGER))
	{
		struct sensor_trigger trig = {.chan = SENSOR_CHAN_ACCEL_XYZ};

		trig.type = SENSOR_TRIG_DATA_READY;
		if (sensor_trigger_set(adxl1362_sens, &trig, trigger_handler))
		{
			printk("SENSOR_TRIG_DATA_READY set error\n");
		}

		trig.type = SENSOR_TRIG_MOTION;
		if (sensor_trigger_set(adxl1362_sens, &trig, trigger_handler))
		{
			printk("SENSOR_TRIG_MOTION set error\n");
		}

		trig.type = SENSOR_TRIG_STATIONARY;
		if (sensor_trigger_set(adxl1362_sens, &trig, trigger_handler))
		{
			printk("SENSOR_TRIG_STATIONARY set error\n");
		}
	}

	while (true)
	{
		if (IS_ENABLED(CONFIG_ADXL362_TRIGGER))
		{
			k_sem_take(&sem, K_FOREVER);
		}

		if (sensor_channel_get(adxl1362_sens, SENSOR_CHAN_ACCEL_X, &accel[0]) < 0)
		{
			printk("Channel get error\n");
			return;
		}

		if (sensor_channel_get(adxl1362_sens, SENSOR_CHAN_ACCEL_Y, &accel[1]) < 0)
		{
			printk("Channel get error\n");
			return;
		}

		if (sensor_channel_get(adxl1362_sens, SENSOR_CHAN_ACCEL_Z, &accel[2]) < 0)
		{
			printk("Channel get error\n");
			return;
		}

		printk("x: %.1f, y: %.1f, z: %.1f (m/s^2)\n",
			   sensor_value_to_double(&accel[0]),
			   sensor_value_to_double(&accel[1]),
			   sensor_value_to_double(&accel[2]));
	}
}