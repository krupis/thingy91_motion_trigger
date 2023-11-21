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


static int ext_sensors_accelerometer_threshold_set(double threshold, bool upper)



static void trigger_handler(const struct device *dev,
							const struct sensor_trigger *trig)
{
	switch (trig->type)
	{
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








static int ext_sensors_accelerometer_threshold_set(double threshold, bool upper)
{
	int err, input_value;
	double range_max_m_s2 = ADXL362_RANGE_MAX_M_S2;

	if ((threshold > range_max_m_s2) || (threshold <= 0.0))
	{
		LOG_ERR("Invalid %s threshold value: %f", upper ? "activity" : "inactivity", threshold);
		return -ENOTSUP;
	}

	/* Convert threshold value into 11-bit decimal value relative
	 * to the configured measuring range of the accelerometer.
	 */
	threshold = (threshold *
				 (ADXL362_THRESHOLD_RESOLUTION_DECIMAL_MAX / range_max_m_s2));

	/* Add 0.5 to ensure proper conversion from double to int. */
	threshold = threshold + 0.5;
	input_value = (int)threshold;

	if (input_value >= ADXL362_THRESHOLD_RESOLUTION_DECIMAL_MAX)
	{
		input_value = ADXL362_THRESHOLD_RESOLUTION_DECIMAL_MAX - 1;
	}
	else if (input_value < 0)
	{
		input_value = 0;
	}

	const struct sensor_value data = {
		.val1 = input_value};

	enum sensor_attribute attr = upper ? SENSOR_ATTR_UPPER_THRESH : SENSOR_ATTR_LOWER_THRESH;

	/* SENSOR_CHAN_ACCEL_XYZ is not supported by the driver in this case. */
	err = sensor_attr_set(adxl1362_sens,
						  SENSOR_CHAN_ACCEL_X,
						  attr,
						  &data);
	if (err)
	{
		LOG_ERR("Failed to set accelerometer threshold value");
		LOG_ERR("Device: %s, error: %d",
				adxl1362_sens->name, err);

		return err;
	}
	return 0;
}