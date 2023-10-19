/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 *
 * Based on luberry pmw3xx drivers https://github.com/slicemk/zmk/compare/main...Luberry:zmk:pmw3389
 */

#define DT_DRV_COMPAT adns3530

#include "adns3530.h"

#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ADNS3530, CONFIG_SENSOR_LOG_LEVEL);

struct adns3530_motion_burst {
    uint8_t motion;
    int8_t dy;
    int8_t dx;
} __attribute__((packed));

static inline int adns3530_cs_select(const struct adns3530_gpio_dt_spec *cs_gpio_cfg,
                                     const uint8_t value) {
    return gpio_pin_set(cs_gpio_cfg->port, cs_gpio_cfg->pin, value);
}

static int adns3530_access(const struct device *dev, const uint8_t reg, uint8_t *value) {
    struct adns3530_data *data = dev->data;
    const struct adns3530_config *cfg = dev->config;
    const struct spi_config *spi_cfg = &cfg->bus_cfg.spi_cfg->spi_conf;
    const struct adns3530_gpio_dt_spec *cs_gpio_cfg = &cfg->bus_cfg.spi_cfg->cs_spec;

    uint8_t access[1] = {reg};
    struct spi_buf_set tx = {
        .buffers =
            &(struct spi_buf){
                .buf = access,
                .len = 1,
            },
        .count = 1,
    };

    uint8_t result[1];
    if (value != NULL) {
        result[0] = *value;
    }
    struct spi_buf_set rx = {
        .buffers =
            &(struct spi_buf){
                .buf = result,
                .len = 1,
            },
        .count = 1,
    };

    adns3530_cs_select(cs_gpio_cfg, 0);
    int err = spi_write(data->bus, spi_cfg, &tx);
    k_sleep(K_USEC(6)); // Tsrad
    if (!err) {
        if ((reg & ADNS3530_WR_MASK)) {
            err = spi_write(data->bus, spi_cfg, &rx);
        } else {
            err = spi_read(data->bus, spi_cfg, &rx);
            if (value != NULL)
                *value = result[0];
        }
    }
    adns3530_cs_select(cs_gpio_cfg, 1);
    return err;
}
static int adns3530_read_reg(const struct device *dev, const uint8_t reg, uint8_t *value) {
    return adns3530_access(dev, reg & ADNS3530_RD_MASK, value);
}
static int adns3530_write_reg(const struct device *dev, const uint8_t reg, const uint8_t value) {
    uint8_t v = value;
    return adns3530_access(dev, reg | ADNS3530_WR_MASK, &v);
}

static int adns3530_read_motion_burst(const struct device *dev,
                                      struct adns3530_motion_burst *burst) {
    struct adns3530_data *data = dev->data;
    const struct adns3530_config *cfg = dev->config;
    const struct spi_config *spi_cfg = &cfg->bus_cfg.spi_cfg->spi_conf;
    const struct adns3530_gpio_dt_spec *cs_gpio_cfg = &cfg->bus_cfg.spi_cfg->cs_spec;

    uint8_t access[1] = {ADNS3530_REG_MTN_BRST};
    struct spi_buf_set tx = {
        .buffers =
            &(struct spi_buf){
                .buf = access,
                .len = 1,
            },
        .count = 1,
    };
    struct spi_buf_set rx = {
        .buffers =
            &(struct spi_buf){
                .buf = (uint8_t *)burst,
                .len = sizeof(struct adns3530_motion_burst),
            },
        .count = 1,
    };
    adns3530_cs_select(cs_gpio_cfg, 0);
    int err = spi_write(data->bus, spi_cfg, &tx);
    k_sleep(K_USEC(6)); // tsrad
    if (!err) {
        err = spi_read(data->bus, spi_cfg, &rx);
    }
    adns3530_cs_select(cs_gpio_cfg, 1);
    return err;
}

#ifdef CONFIG_ADNS3530_TRIGGER
void adns3530_reset_motion(const struct device *dev) {
    // reset motswk interrupt
    adns3530_write_reg(dev, ADNS3530_REG_MOTION, 0x00);
}
#endif

int adns3530_spi_init(const struct device *dev) {
    const struct adns3530_config *cfg = dev->config;
    const struct adns3530_gpio_dt_spec *cs_gpio_cfg = &cfg->bus_cfg.spi_cfg->cs_spec;

    int err;
    err = gpio_pin_configure(cs_gpio_cfg->port, cs_gpio_cfg->pin, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("could configure cs pin %d", err);
        return -EIO;
    }
    return 0;
}

static int adns3530_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct adns3530_data *data = dev->data;
    struct adns3530_motion_burst burst;

    // if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY)
    //     return -ENOTSUP;

    int err = adns3530_read_motion_burst(dev, &burst);
    if (err) {
        return err;
    }
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POS_DX)
        data->dx += (int16_t)burst.dx;
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POS_DY)
        data->dy += (int16_t)burst.dy;
    return 0;
}

static int adns3530_channel_get(const struct device *dev, enum sensor_channel chan,
                                struct sensor_value *val) {
    struct adns3530_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->dx;
        data->dx = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->dy;
        data->dy = 0;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api adns3530_driver_api = {

    .trigger_set = adns3530_trigger_set,
    .sample_fetch = adns3530_sample_fetch,
    .channel_get = adns3530_channel_get,
};

static int adns3530_set_config(const struct device *dev, uint8_t sensor_cpi, uint8_t sensor_rest) {

    return adns3530_write_reg(dev, ADNS3530_REG_CONFIG, sensor_cpi | sensor_rest);
}

static int adns3530_init_chip(const struct device *dev) {
    const struct adns3530_config *const config = dev->config;
    const struct adns3530_gpio_dt_spec *cs_gpio_cfg = &config->bus_cfg.spi_cfg->cs_spec;
    adns3530_cs_select(cs_gpio_cfg, 1);
    k_sleep(K_MSEC(1));

    int err = adns3530_write_reg(dev, ADNS3530_REG_PWR_UP_RST, ADNS3530_RESET_CMD);
    if (err) {
        LOG_ERR("could not reset %d", err);
        return -EIO;
    }
    uint8_t pid = 0x0;
    err = adns3530_read_reg(dev, ADNS3530_REG_PID, &pid);
    if (err) {
        LOG_ERR("could not reset %d", err);
        return -EIO;
    }
    if (pid != ADNS3530_PID) {
        LOG_ERR("pid does not match expected: got (%x), expected(%x)", pid, ADNS3530_PID);
        return -EIO;
    }

    struct adns3530_motion_burst val;
    adns3530_read_motion_burst(dev, &val); // read and throwout initial motion data

    return adns3530_set_config(dev, ADNS3530_PWR_NRML, ADNS3530_CPI_1000);
}

static int adns3530_init(const struct device *dev) {
    const struct adns3530_config *const config = dev->config;
    struct adns3530_data *data = dev->data;

    data->bus = device_get_binding(config->bus_name);
    if (!data->bus) {
        LOG_DBG("master not found: %s", log_strdup(config->bus_name));
        return -EINVAL;
    }

    config->bus_init(dev);

    if (adns3530_init_chip(dev) < 0) {
        LOG_DBG("failed to initialize chip");
        return -EIO;
    }

#ifdef CONFIG_ADNS3530_TRIGGER
    if (adns3530_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }
#endif

    return 0;
}

#define ADNS3530_DATA_SPI(n)                                                                       \
    { .cs_ctrl = {}, }

#define ADNS3530_SPI_CFG(n)                                                                        \
    (&(struct adns3530_spi_cfg){                                                                   \
        .spi_conf =                                                                                \
            {                                                                                      \
                .frequency = DT_INST_PROP(n, spi_max_frequency),                                   \
                .operation =                                                                       \
                    (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA),        \
                .slave = DT_INST_REG_ADDR(n),                                                      \
            },                                                                                     \
        .cs_spec = ADNS3530_GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), cs_gpios, 0),                  \
    })

#define ADNS3530_GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx)                                       \
    {                                                                                              \
        .port = DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(node_id, prop, idx)),                            \
        .pin = DT_GPIO_PIN_BY_IDX(node_id, prop, idx),                                             \
        .dt_flags = DT_GPIO_FLAGS_BY_IDX(node_id, prop, idx),                                      \
    }

#define ADNS3530_CONFIG_SPI(n)                                                                     \
    {                                                                                              \
        .bus_name = DT_INST_BUS_LABEL(n), .bus_init = adns3530_spi_init,                           \
        .bus_cfg = {.spi_cfg = ADNS3530_SPI_CFG(n)},                                               \
        .disable_rest = DT_INST_NODE_HAS_PROP(n, disable_rest),                                    \
        COND_CODE_1(CONFIG_ADNS3530_TRIGGER,                                                       \
                    (, ADNS3530_GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), motswk_gpio, 0)), ())      \
    }

#define ADNS3530_INST(n)                                                                           \
    static struct adns3530_data adns3530_data_##n = ADNS3530_DATA_SPI(n);                          \
    static const struct adns3530_config adns3530_cfg_##n = ADNS3530_CONFIG_SPI(n);                 \
    DEVICE_DT_INST_DEFINE(n, adns3530_init, device_pm_control_nop, &adns3530_data_##n,             \
                          &adns3530_cfg_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,             \
                          &adns3530_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADNS3530_INST)
