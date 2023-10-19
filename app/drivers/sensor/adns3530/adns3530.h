/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Based on luberry pmw3xx drivers https://github.com/slicemk/zmk/compare/main...Luberry:zmk:pmw3389
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADNS3530_H_
#define ZEPHYR_DRIVERS_SENSOR_ADNS3530_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/spi.h>

#define ADNS3530_WR_MASK 0x80
#define ADNS3530_RD_MASK 0x7F

#define ADNS3530_PID 0x0D
#define ADNS3530_REV 0x02

/* General Registers */
#define ADNS3530_REG_PID 0x00
#define ADNS3530_REG_REV_ID 0x01
#define ADNS3530_REG_INV_REV 0x3E
#define ADNS3530_REG_INV_PID 0x3F

#define ADNS3530_REG_SELF_TEST 0x10
#define ADNS3530_REG_CONFIG 0x11
#define ADNS3530_REG_OBSERVE 0x2E

#define ADNS3530_REG_PWR_UP_RST 0x3A

/* Motion Registers */
#define ADNS3530_REG_MOTION 0x02
#define ADNS3530_REG_DELTA_Y 0x03
#define ADNS3530_REG_DELTA_X 0x04

#define ADNS3530_REG_MTN_BRST 0x42

/* Pixel Data Registers */
#define ADNS3530_REG_SQUAL 0x05
#define ADNS3530_REG_SHUT_UP 0x06
#define ADNS3530_REG_SHUT_LO 0x07
#define ADNS3530_REG_MAX_PIX 0x08
#define ADNS3530_REG_PIX_SUM 0x09
#define ADNS3530_REG_MIN_PIX 0x0A

#define ADNS3530_REG_PIX_GRAB 0x0B
#define ADNS3530_REG_CRC0 0x0C
#define ADNS3530_REG_CRC1 0x0D
#define ADNS3530_REG_CRC2 0x0E
#define ADNS3530_REG_CRC3 0x0F

/* Config Settings */
#define ADNS3530_PWR_NRML 0x00
#define ADNS3530_PWR_REST1 0x10
#define ADNS3530_PWR_REST2 0x20
#define ADNS3530_PWR_REST3 0x30

#define ADNS3530_CPI_500 0x00
#define ADNS3530_CPI_1000 0x80

/* power up reset cmd */
#define ADNS3530_RESET_CMD 0x5A

struct adns3530_gpio_dt_spec {
    const struct device *port;
    gpio_pin_t pin;
    gpio_dt_flags_t dt_flags;
};

struct adns3530_spi_cfg {
    struct spi_config spi_conf;
    struct adns3530_gpio_dt_spec cs_spec;
};

union adns3530_bus_cfg {
    struct adns3530_spi_cfg *spi_cfg;
};

struct adns3530_config {
    char *bus_name;
    int (*bus_init)(const struct device *dev);
    const union adns3530_bus_cfg bus_cfg;
    bool disable_rest;
    int cpi;
#if CONFIG_ADNS3530_TRIGGER
    struct adns3530_gpio_dt_spec motswk_spec;
#endif // CONFIG_ADNS3530_TRIGGER
};

struct adns3530_transfer_function {
    int (*read_data)(const struct device *dev, int16_t *value);
};

struct adns3530_data {
    const struct device *bus;
    struct spi_cs_control cs_ctrl;

    int16_t dx;
    int16_t dy;

    const struct adns3530_transfer_function *hw_tf;

#ifdef CONFIG_ADNS3530_TRIGGER

    struct gpio_callback motswk_gpio_cb;
    const struct device *dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;

#if defined(CONFIG_ADNS3530_TRIGGER_OWN_THREAD)
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_ADNS3530_THREAD_STACK_SIZE);
    struct k_sem gpio_sem;
    struct k_thread thread;
#elif defined(CONFIG_ADNS3530_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif

#endif /* CONFIG_ADNS3530_TRIGGER */
};

int adns3530_spi_init(const struct device *dev);
#ifdef CONFIG_ADNS3530_TRIGGER

int adns3530_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                         sensor_trigger_handler_t handler);

int adns3530_init_interrupt(const struct device *dev);

void adns3530_reset_motion(const struct device *dev);
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_ADNS3530_H_ */
