#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define LOG_LEVEL 4
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);

struct sensor_value data[3];

/* Convert to s/m2 depending on the maximum measured range used for adxl362. */
#if IS_ENABLED(CONFIG_ADXL362_ACCEL_RANGE_2G)
#define ADXL362_RANGE_MAX_M_S2 19.6133
#elif IS_ENABLED(CONFIG_ADXL362_ACCEL_RANGE_4G)
#define ADXL362_RANGE_MAX_M_S2 39.2266
#elif IS_ENABLED(CONFIG_ADXL362_ACCEL_RANGE_8G)
#define ADXL362_RANGE_MAX_M_S2 78.4532
#endif

/* This is derived from the sensitivity values in the datasheet. */
#define ADXL362_THRESHOLD_RESOLUTION_DECIMAL_MAX 2000

#if IS_ENABLED(CONFIG_ADXL362_ACCEL_ODR_12_5)
#define ADXL362_TIMEOUT_MAX_S 5242.88
#elif IS_ENABLED(CONFIG_ADXL362_ACCEL_ODR_25)
#define ADXL362_TIMEOUT_MAX_S 2621.44
#elif IS_ENABLED(CONFIG_ADXL362_ACCEL_ODR_50)
#define ADXL362_TIMEOUT_MAX_S 1310.72
#elif IS_ENABLED(CONFIG_ADXL362_ACCEL_ODR_100)
#define ADXL362_TIMEOUT_MAX_S 655.36
#elif IS_ENABLED(CONFIG_ADXL362_ACCEL_ODR_200)
#define ADXL362_TIMEOUT_MAX_S 327.68
#elif IS_ENABLED(CONFIG_ADXL362_ACCEL_ODR_400)
#define ADXL362_TIMEOUT_MAX_S 163.84
#endif

#define ADXL362_TIMEOUT_RESOLUTION_MAX 65536

#define DEFAULT_ADXL362_NODE DT_ALIAS(adxl362)
BUILD_ASSERT(DT_NODE_HAS_STATUS(DEFAULT_ADXL362_NODE, okay),
			 "ADXL362 not specified in DT");

// DEVICE TREE STRUCTURE
const struct device *adxl1362_sens = DEVICE_DT_GET(DEFAULT_ADXL362_NODE);

static int ext_sensors_accelerometer_threshold_set(double threshold, bool upper);

static void trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	int err = 0;
	switch (trig->type)
	{

	case SENSOR_TRIG_MOTION:
	case SENSOR_TRIG_STATIONARY:

		if (sensor_sample_fetch(dev) < 0)
		{
			LOG_ERR("Sample fetch error");
			return;
		}

		err = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, &data[0]);
		if (err)
		{
			LOG_ERR("sensor_channel_get, error: %d", err);
			return;
		}

		printk("x: %.1f, y: %.1f, z: %.1f (m/s^2)\n",
			   sensor_value_to_double(&data[0]),
			   sensor_value_to_double(&data[1]),
			   sensor_value_to_double(&data[2]));

		if (trig->type == SENSOR_TRIG_MOTION)
		{
			LOG_DBG("Activity detected");
		}
		else
		{
			LOG_DBG("Inactivity detected");
		}

		break;
	default:
		LOG_ERR("Unknown trigger: %d", trig->type);
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
		printk("Configuring triggers\n");
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
		ext_sensors_accelerometer_threshold_set(5.0, true);
		ext_sensors_accelerometer_threshold_set(0.5, false);
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

	printf("threshold value to be set : %d\n", data.val1);

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