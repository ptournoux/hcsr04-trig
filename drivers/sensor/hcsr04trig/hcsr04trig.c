/*
 * Copyright (c) 2024 Adrien Leravat
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hcsr04trig

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(HCSR04TRIG, CONFIG_SENSOR_LOG_LEVEL);

#define HC_SR04_MM_PER_MS     171

#define GROVE_ULTRASONIC_SENSOR 1

static const uint32_t hw_cycles_per_ms = sys_clock_hw_cycles_per_sec() / 1000;

struct hcsr04trig_data {
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_sem sem;
	uint32_t start_cycles;
	atomic_t echo_high_cycles;
};

struct hcsr04trig_config {
	struct gpio_dt_spec trigger_gpios;
};

static void hcsr04trig_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

static int hcsr04trig_configure_gpios(const struct hcsr04trig_config *cfg)
{
	int ret;

	if (!gpio_is_ready_dt(&cfg->trigger_gpios)) {
		LOG_ERR("GPIO '%s' not ready", cfg->trigger_gpios.port->name);
		return -ENODEV;
	}
	ret = gpio_pin_configure_dt(&cfg->trigger_gpios, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure '%s' as output: %d", cfg->trigger_gpios.port->name,
			ret);
		return ret;
	}
	return 0;
}

static int hcsr04trig_configure_interrupt(const struct hcsr04trig_config *cfg, struct hcsr04trig_data *data)
{
	int ret;

	/* Disable initially to avoid spurious interrupts. */
	ret = gpio_pin_interrupt_configure(cfg->trigger_gpios.port, cfg->trigger_gpios.pin,
		GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("Failed to configure '%s' as interrupt: %d", cfg->trigger_gpios.port->name,
			ret);
		return -EIO;
	}
	gpio_init_callback(&data->gpio_cb, &hcsr04trig_gpio_callback, BIT(cfg->trigger_gpios.pin));
	ret = gpio_add_callback(cfg->trigger_gpios.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add callback on '%s': %d", cfg->trigger_gpios.port->name, ret);
		return -EIO;
	}
	return 0;
}

static int hcsr04trig_init(const struct device *dev)
{
	const struct hcsr04trig_config *cfg = dev->config;
	struct hcsr04trig_data *data = dev->data;
	int ret;
	k_sem_init(&data->sem, 0, 1);
	ret = hcsr04trig_configure_gpios(cfg);
	if (ret < 0) {
		return ret;
	}
	return 0;
}

static void hcsr04trig_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct hcsr04trig_data *data = CONTAINER_OF(cb, struct hcsr04trig_data, gpio_cb);
	const struct hcsr04trig_config *cfg = data->dev->config;
	//LOG_ERR("CB ");
	if (gpio_pin_get(dev, cfg->trigger_gpios.pin) == 1) {
		data->start_cycles = k_cycle_get_32();
	} else {
		atomic_set(&data->echo_high_cycles, k_cycle_get_32() - data->start_cycles);
		gpio_pin_interrupt_configure_dt(&cfg->trigger_gpios, GPIO_INT_DISABLE);
		k_sem_give(&data->sem);
	}
}

static int hcsr04trig_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct hcsr04trig_config *cfg = dev->config;
	struct hcsr04trig_data *data = dev->data;
	int ret;
	ret = gpio_pin_configure_dt(&cfg->trigger_gpios, GPIO_OUTPUT_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to configure '%s' as output: %d", cfg->trigger_gpios.port->name,
			ret);
		return ret;
	}
	

	/* Generate 10us trigger */
	ret = gpio_pin_set_dt(&cfg->trigger_gpios, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set trigger pin: %d", ret);
		return ret;
	}

	k_busy_wait(10);

	ret = gpio_pin_set_dt(&cfg->trigger_gpios, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set trigger pin: %d", ret);
		return ret;
	}
	// Now change PIN config to input
	ret = gpio_pin_configure_dt(&cfg->trigger_gpios, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to set configure pin as input: %d", ret);
		return ret;
	}
	ret = hcsr04trig_configure_interrupt(cfg, data);
	if (ret < 0) {
		LOG_ERR("Failed to set configure pin as interrupt: %d", ret);
		return ret;
	}
	ret = gpio_pin_interrupt_configure_dt(&cfg->trigger_gpios, GPIO_INT_EDGE_BOTH);
	if (ret < 0) {
		LOG_ERR("Failed to set configure echo pin as interrupt w rising and falling edges: %d", ret);
		return ret;
	}
	
	if (k_sem_take(&data->sem, K_MSEC(10)) != 0) {
		LOG_ERR("Echo signal was not received");
		return -EIO;
	}
	return 0;
}

static int hcsr04trig_channel_get(const struct device *dev, enum sensor_channel chan,
	struct sensor_value *val)
{
	const struct hcsr04trig_data *data = dev->data;
	uint32_t distance_mm;

	if (chan != SENSOR_CHAN_DISTANCE) {
		return -ENOTSUP;
	}

	distance_mm = HC_SR04_MM_PER_MS * atomic_get(&data->echo_high_cycles) /
			hw_cycles_per_ms;
	return sensor_value_from_milli(val, distance_mm);
}

static DEVICE_API(sensor, hcsr04trig_driver_api) = {
	.sample_fetch = hcsr04trig_sample_fetch,
	.channel_get = hcsr04trig_channel_get
};


#define HCSR04TRIG_INIT(index)                                                           \
	static struct hcsr04trig_data hcsr04trig_data_##index = {                             \
		.dev = DEVICE_DT_INST_GET(index),                                     \
		.start_cycles = 0,                                                    \
		.echo_high_cycles = ATOMIC_INIT(0),                                   \
	};                                                                            \
	static struct hcsr04trig_config hcsr04trig_config_##index = {                         \
		.trigger_gpios = GPIO_DT_SPEC_INST_GET(index, trigger_gpios),         \
	};                                                                            \
                                                                                      \
	SENSOR_DEVICE_DT_INST_DEFINE(index, &hcsr04trig_init, NULL, &hcsr04trig_data_##index, \
				&hcsr04trig_config_##index, POST_KERNEL,                  \
				CONFIG_SENSOR_INIT_PRIORITY, &hcsr04trig_driver_api);     \

DT_INST_FOREACH_STATUS_OKAY(HCSR04TRIG_INIT)
