// Copyright (c) 2023 Sava N
// SPDX-License-Identifier: MIT
// Based on drivers written by caksoylar
// https://github.com/caksoylar/zmk-config/blob/main/config/boards/shields/rgbled_widget/leds.c

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/bluetooth/services/bas.h>

#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/events/ble_active_profile_changed.h>
#include <zmk/split/bluetooth/peripheral.h>
#include <zmk/events/battery_state_changed.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define LED_GPIO_NODE_ID DT_COMPAT_GET_ANY_STATUS_OKAY(gpio_leds)

// GPIO-based LED device and indices of LEDs inside its DT node
static const struct device *led_dev = DEVICE_DT_GET(LED_GPIO_NODE_ID);

#define LED_POWER_RED DT_NODE_CHILD_IDX(DT_ALIAS(led_pwr_r))
#define LED_POWER_GREEN DT_NODE_CHILD_IDX(DT_ALIAS(led_pwr_g))
#define LED_BT_0 DT_NODE_CHILD_IDX(DT_ALIAS(led_bt_0))
#define LED_BT_1 DT_NODE_CHILD_IDX(DT_ALIAS(led_bt_1))
#define LED_BT_2 DT_NODE_CHILD_IDX(DT_ALIAS(led_bt_2))

struct blink_item {
    uint8_t led_id;
    int8_t num_blinks;
};

// define message queue of blink work items, that will be processed by a separate thread
K_MSGQ_DEFINE(led_msgq, sizeof(struct blink_item), 16, 4);

#if IS_ENABLED(CONFIG_ZMK_BLE)
static int led_peripheral_listener_cb(const zmk_event_t *eh) {

    // reset message queue. we only support one led blinking at a time
    k_msgq_purge(&led_msgq);

    struct blink_item blink;

    switch (zmk_ble_active_profile_index()) {
    case 0:
        blink.led_id = LED_BT_0;
        break;
    case 1:
        blink.led_id = LED_BT_1;
        break;
    case 2:
        blink.led_id = LED_BT_2;
        break;
    default:
        return -1;
    }

    if (zmk_ble_active_profile_is_connected()) {
        blink.num_blinks = 3;
    } else if (zmk_ble_active_profile_is_open()) {
        blink.num_blinks = -1;
    } else {
        blink.num_blinks = -1;
    }

    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);
    return 0;
}

// run led_peripheral_listener_cb on profile status change event
ZMK_LISTENER(led_peripheral_listener, led_peripheral_listener_cb);
ZMK_SUBSCRIPTION(led_peripheral_listener, zmk_ble_active_profile_changed);
#endif // IS_ENABLED(CONFIG_ZMK_BLE)

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
static int led_battery_listener_cb(const zmk_event_t *eh) {
    // check if we are in critical battery levels at state change
    uint8_t battery_level = ((struct zmk_battery_state_changed *)eh)->state_of_charge;

    if (battery_level <= CONFIG_LED_WIDGET_BATTERY_LEVEL_CRITICAL) {
        led_on(led_dev, LED_POWER_RED);
    } else {
        led_off(led_dev, LED_POWER_RED);
    }
    return 0;
}

// run led_battery_listener_cb on battery state change event
ZMK_LISTENER(led_battery_listener, led_battery_listener_cb);
ZMK_SUBSCRIPTION(led_battery_listener, zmk_battery_state_changed);
#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)

extern void led_blink_thread(void *d0, void *d1, void *d2) {
    ARG_UNUSED(d0);
    ARG_UNUSED(d1);
    ARG_UNUSED(d2);

    while (true) {
        // wait until a blink item is received and process it
        struct blink_item blink;
        k_msgq_get(&led_msgq, &blink, K_FOREVER);

        if (blink.num_blinks == 0) {
            continue;
        }

        LOG_DBG("Got a blink item from msgq");

        led_on(led_dev, blink.led_id);

        // wait for blink duration
        k_sleep(K_MSEC(CONFIG_LED_WIDGET_INTERVAL_MS));

        led_off(led_dev, blink.led_id);

        if (blink.num_blinks > 0) {
            blink.num_blinks--;
        }

        k_msgq_put(&led_msgq, &blink, K_NO_WAIT);

        // wait interval before processing another blink
        k_sleep(K_MSEC(CONFIG_LED_WIDGET_INTERVAL_MS));
    }
}

// define led_thread with stack size 512, start running it 500 ms after boot
K_THREAD_DEFINE(led_tid, 512, led_blink_thread, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO,
                0, 500);

static int leds_init(const struct device *device) {
    if (!device_is_ready(led_dev)) {
        LOG_ERR("Device %s is not ready", led_dev->name);
        return -ENODEV;
    }

    // blink green to indicate keyboard is on
    struct blink_item blink = {.num_blinks = 3, .led_id = LED_POWER_GREEN};
    k_msgq_put(&led_msgq, &blink, K_NO_WAIT);

    return 0;
}

// run leds_init on boot
SYS_INIT(leds_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
