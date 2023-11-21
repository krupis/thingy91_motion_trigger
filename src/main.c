#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

struct sensor_value data[3];

#define DEFAULT_ADXL362_NODE DT_ALIAS(adxl362)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_ADXL362_NODE, okay),
			 "ADXL362 not specified in DT");

// DEVICE TREE STRUCTURE
const struct device *adxl1362_sens = DEVICE_DT_GET(DEFAULT_ADXL362_NODE);

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	int err = 0;
	switch (trig->type)
	{
	case SENSOR_TRIG_DATA_READY:
	{
		printk("SENSOR_TRIG_DATA_READY\n");
		break;
	}

	case SENSOR_TRIG_MOTION:
	{
		if (sensor_sample_fetch(dev) < 0)
		{
			printk("Sample fetch error \n");
			return;
		}

		err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &data[0]);
		if (err)
		{
			printk("sensor_channel_get, error: %d \n", err);
			return;
		}

		printk("x: %.1f, y: %.1f, z: %.1f (m/s^2)\n",
			   sensor_value_to_double(&data[0]),
			   sensor_value_to_double(&data[1]),
			   sensor_value_to_double(&data[2]));

		printk("Inactivity detected\n");
		break;
	}

	case SENSOR_TRIG_STATIONARY:
	{
		if (sensor_sample_fetch(dev) < 0)
		{
			printk("Sample fetch error \n");
			return;
		}

		err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &data[0]);
		if (err)
		{
			printk("sensor_channel_get, error: %d \n", err);
			return;
		}

		printk("x: %.1f, y: %.1f, z: %.1f (m/s^2)\n",
			   sensor_value_to_double(&data[0]),
			   sensor_value_to_double(&data[1]),
			   sensor_value_to_double(&data[2]));

		printk("Activity detected\n");
		break;
	}

	default:
		printk("Unknown trigger\n");
	}
}

void main(void)
{

	if (!device_is_ready(adxl1362_sens))
	{
		printk("sensor: device %s not ready.\n", adxl1362_sens->name);
		return 0;
	}

	if (IS_ENABLED(CONFIG_ADXL362_TRIGGER))
	{
		struct sensor_trigger trig_motion = {
			.chan = SENSOR_CHAN_ACCEL_XYZ,
			.type = SENSOR_TRIG_MOTION,
		};
		if (sensor_trigger_set(adxl1362_sens, &trig_motion, trigger_handler))
		{
			printk("SENSOR_TRIG_MOTION set error\n");
		}
		struct sensor_trigger trig_stationary = {
			.chan = SENSOR_CHAN_ACCEL_XYZ,
			.type = SENSOR_TRIG_STATIONARY,
		};
		if (sensor_trigger_set(adxl1362_sens, &trig_stationary, trigger_handler))
		{
			printk("SENSOR_TRIG_STATIONARY set error\n");
		}
	}
}