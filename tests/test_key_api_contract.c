/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "key.h"
#include "gpiod.h"

struct key_event_counts {
    int pressed;
    int released;
    int click;
    int double_click;
    int long_press;
    int hold_repeat;
};

static int g_failures;

#define CHECK_TRUE(expr) do { \
    if (!(expr)) { \
        printf("FAIL:%s:%d: expected true: %s\n", __FILE__, __LINE__, #expr); \
        g_failures++; \
    } \
} while (0)

#define CHECK_INT_EQ(actual, expected) do { \
    int _actual = (int)(actual); \
    int _expected = (int)(expected); \
    if (_actual != _expected) { \
        printf("FAIL:%s:%d: expected %s == %d, got %d\n", \
            __FILE__, __LINE__, #actual, _expected, _actual); \
        g_failures++; \
    } \
} while (0)

static void reset_test_state(void)
{
    g_failures = 0;
    fake_gpiod_reset();
}

static void key_cb(struct key_handle *key, key_event_t event, void *user_data)
{
    struct key_event_counts *counts = user_data;

    CHECK_TRUE(key != NULL);
    CHECK_TRUE(counts != NULL);

    if (!counts)
        return;

    switch (event) {
    case KEY_EV_PRESSED:
        counts->pressed++;
        break;
    case KEY_EV_RELEASED:
        counts->released++;
        break;
    case KEY_EV_CLICK:
        counts->click++;
        break;
    case KEY_EV_DOUBLE_CLICK:
        counts->double_click++;
        break;
    case KEY_EV_LONG_PRESS:
        counts->long_press++;
        break;
    case KEY_EV_HOLD_REPEAT:
        counts->hold_repeat++;
        break;
    }
}

static void test_error_paths(void)
{
    key_config_t config = {
        .gpio_num = 5,
        .active_low = 0,
        .long_press_ms = 100,
        .double_click_ms = 80,
    };

    CHECK_TRUE(key_add_gpio(&config, key_cb, NULL) == NULL);
    CHECK_INT_EQ(key_service_start(), 0);
    CHECK_INT_EQ(key_service_start(), 0);
    CHECK_TRUE(key_add_gpio(NULL, key_cb, NULL) == NULL);
    CHECK_TRUE(key_add_gpio(&config, NULL, NULL) == NULL);
    config.gpio_num = -1;
    CHECK_TRUE(key_add_gpio(&config, key_cb, NULL) == NULL);
    key_remove(NULL);
    key_service_stop();
    key_service_stop();
}

static void test_functional(void)
{
    key_config_t config = {
        .gpio_num = 5,
        .active_low = 0,
        .long_press_ms = 45,
        .double_click_ms = 80,
    };
    struct key_event_counts counts;
    struct key_handle *key;

    memset(&counts, 0, sizeof(counts));
    fake_gpiod_set_value(config.gpio_num, 0);

    CHECK_INT_EQ(key_service_start(), 0);
    key = key_add_gpio(&config, key_cb, &counts);
    CHECK_TRUE(key != NULL);
    if (!key) {
        key_service_stop();
        return;
    }

    usleep(40 * 1000);
    fake_gpiod_set_value(config.gpio_num, 1);
    usleep(110 * 1000);
    fake_gpiod_set_value(config.gpio_num, 0);
    usleep(80 * 1000);

    CHECK_TRUE(counts.pressed >= 1);
    CHECK_TRUE(counts.long_press >= 1);
    CHECK_TRUE(counts.released >= 1);
    CHECK_INT_EQ(counts.double_click, 0);

    key_remove(key);
    key_service_stop();
    CHECK_INT_EQ(fake_gpiod_get_release_count(), 1);
}

static int finish_test(const char *name)
{
    if (g_failures != 0) {
        printf("%s FAILED: %d failure(s)\n", name, g_failures);
        return 1;
    }
    printf("%s PASSED\n", name);
    return 0;
}

int main(int argc, char **argv)
{
    const char *mode = (argc > 1) ? argv[1] : "all";

    if (strcmp(mode, "functional") == 0) {
        reset_test_state();
        test_functional();
        return finish_test("key api functional test");
    }
    if (strcmp(mode, "error-paths") == 0) {
        reset_test_state();
        test_error_paths();
        return finish_test("key api error paths test");
    }
    if (strcmp(mode, "all") == 0) {
        reset_test_state();
        test_functional();
        if (finish_test("key api functional test") != 0)
            return 1;
        reset_test_state();
        test_error_paths();
        if (finish_test("key api error paths test") != 0)
            return 1;
        printf("key api contract test PASSED\n");
        return 0;
    }

    fprintf(stderr, "usage: %s [all|functional|error-paths]\n", argv[0]);
    return 2;
}
