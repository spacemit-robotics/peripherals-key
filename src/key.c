/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <sys/epoll.h>
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <errno.h>

#include <key.h>

/* --- 内部常量定义 --- */
#define DEFAULT_LONG_PRESS_MS     1500
#define DEFAULT_DOUBLE_CLICK_MS   300
#define DEFAULT_REPEAT_MS         200     // 长按连发间隔
#define DEBOUNCE_MS               20      // 防抖时间
#define SCAN_INTERVAL_MS          10      // 扫描间隔

#define KEY_STATE_RELEASED        0
#define KEY_STATE_PRESSED         1

#define MAX_KEYS                  32      // 最大按键数量

// #define KEY_DEBUG
// #define KEY_INFO
#define KEY_ERROR

#ifdef KEY_DEBUG
#define key_log_debug(...) printf("%s,%d,debug:", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define key_log_debug(...)
#endif

#ifdef KEY_INFO
#define key_log_info(...) printf("%s,%d,info:", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define key_log_info(...)
#endif

#ifdef KEY_ERROR
#define key_log_err(...) printf("%s,%d,err:", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define key_log_err(...)
#endif





/* --- 内部数据结构 --- */
typedef struct {
    int gpio_num;               // GPIO 编号
    int active_low;             // 是否低电平有效
    int gpio_fd;                // GPIO 文件描述符
    int current_state;          // 当前物理状态 (去抖后)
    int last_stable_state;      // 上一稳定状态

    // 时间相关
    uint64_t press_timestamp;    // 按下时间戳 (ns)
    uint64_t release_timestamp;  // 释放时间戳 (ns)
    uint64_t last_event_time;    // 上次事件时间 (防抖用)

    // 事件检测
    int click_count;            // 点击计数 (用于检测双击)
    uint64_t first_click_time;   // 第一次点击时间
    int long_press_detected;     // 长按已触发标志
    int hold_repeat_count;       // 连发计数

    // 配置参数 (ms)
    int long_press_ms;
    int double_click_ms;
    int repeat_ms;               // 连发间隔

    // 回调
    key_callback_t callback;
    void *user_data;

    // 链表
    struct key_handle *next;
} key_internal_t;

struct key_handle {
    key_internal_t internal;
    struct gpiod_chip *chip;   // GPIO芯片句柄
#if defined(LIBGPIOD_V2)
    unsigned int offset;
    struct gpiod_line_settings *settings;
    struct gpiod_line_config *lcfg;
    struct gpiod_request_config *rcfg;
    struct gpiod_line_request *req;
#else
    struct gpiod_line *line;    // libgpiod 句柄
#endif
    int chip_fd;                // GPIO芯片文件描述符
    struct key_handle *next;
};

/* --- 全局变量 --- */
static struct key_handle *key_list = NULL;
static pthread_mutex_t key_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_t scan_thread;
static volatile int scan_thread_running = 0;
static volatile int service_started = 0;
static int epoll_fd = -1;

/* --- 内部函数声明 --- */
static void *key_scan_thread(void *arg);
static uint64_t get_timestamp_ns(void);
static int ms_to_ns(int ms);
static int read_gpio_state(struct key_handle *key);
static void process_key_event(struct key_handle *key);
static void detect_key_events(struct key_handle *key, int new_state, uint64_t event_time);
static void fire_key_event(struct key_handle *key, key_event_t event);
#if defined(LIBGPIOD_V2)
static struct gpiod_chip* open_gpio_chip(int gpio_num, unsigned int *offset);
#else
static struct gpiod_chip* open_gpio_chip(int gpio_num);
#endif

/* ============================================================
 * 公共 API 实现
 * ============================================================ */

/**
 * @brief 全局初始化 (启动后台扫描线程)
 */
int key_service_start(void)
{
    if (service_started) {
        return 0;  // 已经启动
    }

    key_log_info("按键服务启动...\n");

    // 初始化epoll（用于GPIO事件监测）
    epoll_fd = epoll_create1(0);
    if (epoll_fd < 0) {
        perror("epoll_create1 failed");
        return -1;
    }

    // 创建扫描线程
    scan_thread_running = 1;
    if (pthread_create(&scan_thread, NULL, key_scan_thread, NULL) != 0) {
        perror("创建扫描线程失败");
        close(epoll_fd);
        return -1;
    }

    service_started = 1;
    key_log_info("按键服务启动成功\n");
    return 0;
}

/**
 * @brief 全局销毁 (停止线程，清理资源)
 */
void key_service_stop(void)
{
    if (!service_started) {
        return;
    }

    key_log_info("停止按键服务...\n");

    // 停止扫描线程
    scan_thread_running = 0;
    if (scan_thread) {
        pthread_join(scan_thread, NULL);
    }

    // 关闭epoll
    if (epoll_fd >= 0) {
        close(epoll_fd);
        epoll_fd = -1;
    }

    // 清理所有按键
    pthread_mutex_lock(&key_mutex);
    struct key_handle *current = key_list;
    while (current) {
        struct key_handle *next = current->next;
        key_remove(current);
        current = next;
    }
    key_list = NULL;
    pthread_mutex_unlock(&key_mutex);

    service_started = 0;
    key_log_info("按键服务已停止\n");
}

/**
 * @brief 添加一个 GPIO 按键
 */
struct key_handle *key_add_gpio(const key_config_t *config, key_callback_t cb, void *user_data)
{
    if (!config || !cb) {
        fprintf(stderr, "无效参数\n");
        return NULL;
    }

    if (!service_started) {
        fprintf(stderr, "请先调用 key_service_start()\n");
        return NULL;
    }

    key_log_info("添加按键: GPIO%d, active_low=%d\n",
        config->gpio_num, config->active_low);

    // 分配内存
    struct key_handle *key = malloc(sizeof(struct key_handle));
    if (!key) {
        perror("分配内存失败");
        return NULL;
    }
    memset(key, 0, sizeof(struct key_handle));

    // 填充配置
    key->internal.gpio_num = config->gpio_num;
    key->internal.active_low = config->active_low;
    key->internal.long_press_ms = (config->long_press_ms > 0) ?
        config->long_press_ms : DEFAULT_LONG_PRESS_MS;
    key->internal.double_click_ms = (config->double_click_ms > 0) ?
        config->double_click_ms : DEFAULT_DOUBLE_CLICK_MS;
    key->internal.repeat_ms = DEFAULT_REPEAT_MS;
    key->internal.callback = cb;
    key->internal.user_data = user_data;

    // 初始化状态
    key->internal.current_state = KEY_STATE_RELEASED;
    key->internal.last_stable_state = KEY_STATE_RELEASED;
    key->internal.click_count = 0;
    key->internal.long_press_detected = 0;
    key->internal.hold_repeat_count = 0;
    key->internal.press_timestamp = 0;
    key->internal.release_timestamp = 0;
    key->internal.last_event_time = 0;
    key->internal.first_click_time = 0;

    // 使用 libgpiod 打开 GPIO
#if defined(LIBGPIOD_V2)
    unsigned int offset = 0;
    struct gpiod_chip *chip = open_gpio_chip(config->gpio_num, &offset);
#else
    struct gpiod_chip *chip = open_gpio_chip(config->gpio_num);
#endif

    if (!chip) {
        fprintf(stderr, "无法打开GPIO芯片\n");
        free(key);
        return NULL;
    }
    key->chip = chip;
#if defined(LIBGPIOD_V2)
    key->offset = offset;
#endif

#if defined(LIBGPIOD_V2)
    key->settings = gpiod_line_settings_new();
    if (!key->settings) {
        fprintf(stderr, "Failed to create line settings\n");
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }
    gpiod_line_settings_set_direction(key->settings, GPIOD_LINE_DIRECTION_INPUT);
    if (config->active_low) {
        gpiod_line_settings_set_bias(key->settings, GPIOD_LINE_BIAS_PULL_UP);
    } else {
        gpiod_line_settings_set_bias(key->settings, GPIOD_LINE_BIAS_PULL_DOWN);
    }

    key->lcfg = gpiod_line_config_new();
    if (!key->lcfg) {
        fprintf(stderr, "Failed to create line config\n");
        gpiod_line_settings_free(key->settings);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }

    unsigned int _offset = key->offset;
    if (gpiod_line_config_add_line_settings(key->lcfg, &_offset, 1, key->settings) < 0) {
        fprintf(stderr, "Failed to add line settings to config\n");
        gpiod_line_config_free(key->lcfg);
        gpiod_line_settings_free(key->settings);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }

    key->rcfg = gpiod_request_config_new();
    if (!key->rcfg) {
        fprintf(stderr, "Failed to create request config\n");
        gpiod_line_config_free(key->lcfg);
        gpiod_line_settings_free(key->settings);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }
    gpiod_request_config_set_consumer(key->rcfg, "key_driver");

    key->req = gpiod_chip_request_lines(chip, key->rcfg, key->lcfg);
    if (!key->req) {
        fprintf(stderr, "Failed to request lines\n");
        gpiod_request_config_free(key->rcfg);
        gpiod_line_config_free(key->lcfg);
        gpiod_line_settings_free(key->settings);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }

    enum gpiod_line_value v = gpiod_line_request_get_value(key->req, key->offset);
    if ((int)v < 0) {
        fprintf(stderr, "读取GPIO状态失败\n");
        gpiod_line_request_release(key->req);
        gpiod_request_config_free(key->rcfg);
        gpiod_line_config_free(key->lcfg);
        gpiod_line_settings_free(key->settings);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }
    int value = (v == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
#else
    // 获取GPIO线
    int offset = config->gpio_num;  // 简化处理，实际需要映射
    // key->line = gpiod_chip_get_line(chip, config->gpio_num);
    key->line = gpiod_chip_get_line(chip, offset);
    if (!key->line) {
        fprintf(stderr, "无法获取GPIO线 %d\n", config->gpio_num);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }

    // 配置为输入模式
    struct gpiod_line_request_config req_cfg = {
        .consumer = "key_driver",
        .request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT,
        .flags = 0
    };

    // 设置偏置
    if (config->active_low) {
        // 低电平有效，通常使用上拉电阻
        req_cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP;
    } else {
        // 高电平有效，通常使用下拉电阻
        req_cfg.flags |= GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN;
    }

    if (gpiod_line_request(key->line, &req_cfg, 0) < 0) {
        fprintf(stderr, "申请GPIO线失败: %d\n", config->gpio_num);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }

    // 读取初始状态
    int value = gpiod_line_get_value(key->line);
    if (value < 0) {
        fprintf(stderr, "读取GPIO状态失败\n");
        gpiod_line_release(key->line);
        gpiod_chip_close(chip);
        free(key);
        return NULL;
    }
#endif

    // 转换为逻辑状态
    if (config->active_low) {
        key->internal.current_state = (value == 0) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
    } else {
        key->internal.current_state = (value == 1) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
    }
    key->internal.last_stable_state = key->internal.current_state;

    // 添加到链表
    pthread_mutex_lock(&key_mutex);
    key->next = key_list;
    key_list = key;
    pthread_mutex_unlock(&key_mutex);

    key_log_info("按键 GPIO%d 添加成功\n", config->gpio_num);
    return key;
}

/**
 * @brief 删除一个按键
 */
void key_remove(struct key_handle *key)
{
    if (!key) {
        return;
    }

    key_log_info("删除按键: GPIO%d\n", key->internal.gpio_num);

    pthread_mutex_lock(&key_mutex);

    // 从链表中移除
    struct key_handle **pp = &key_list;
    while (*pp) {
        if (*pp == key) {
            *pp = key->next;
            break;
        }
        pp = &(*pp)->next;
    }

    pthread_mutex_unlock(&key_mutex);

    // 释放GPIO资源
#if defined(LIBGPIOD_V2)
    if (key->req) {
        gpiod_line_request_release(key->req);
    }
    if (key->rcfg) {
        gpiod_request_config_free(key->rcfg);
    }
    if (key->lcfg) {
        gpiod_line_config_free(key->lcfg);
    }
    if (key->settings) {
        gpiod_line_settings_free(key->settings);
    }
#else
    if (key->line) {
        gpiod_line_release(key->line);
    }
#endif

    if (key->chip) {
        gpiod_chip_close(key->chip);
    }

    free(key);
}

/* ============================================================
 * 内部函数实现
 * ============================================================ */

/**
 * @brief 扫描线程主函数
 */
static void *key_scan_thread(void *arg)
{
    (void)arg;

    key_log_info("按键扫描线程启动\n");

    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = SCAN_INTERVAL_MS * 1000000;  // 转换为ns

    while (scan_thread_running) {
        // 扫描所有按键
        pthread_mutex_lock(&key_mutex);
        struct key_handle *key = key_list;
        while (key) {
            process_key_event(key);
            key = key->next;
        }
        pthread_mutex_unlock(&key_mutex);

        // 休眠
        nanosleep(&ts, NULL);
    }

    key_log_info("按键扫描线程退出\n");
    return NULL;
}

/**
 * @brief 处理单个按键事件
 */
static void process_key_event(struct key_handle *key)
{
    if (!key) {
        return;
    }

    // 读取GPIO状态
    int raw_state = read_gpio_state(key);
    if (raw_state < 0) {
        return;  // 读取失败
    }

    // 防抖处理
    uint64_t now = get_timestamp_ns();
    uint64_t debounce_ns = ms_to_ns(DEBOUNCE_MS);

    // 状态发生变化
    if (raw_state != key->internal.last_stable_state) {
        key->internal.last_event_time = now;
        key->internal.last_stable_state = raw_state;
    }

    // 检查是否已稳定
    uint64_t time_since_change = now - key->internal.last_event_time;

    if (time_since_change > debounce_ns) {
        // 状态已稳定

        // 检查是否需要处理状态变化
        if (raw_state != key->internal.current_state) {
            key_log_info("GPIO%d 状态变化: %d -> %d (稳定时间: %luns)\n",
                key->internal.gpio_num,
                key->internal.current_state,
                raw_state,
                time_since_change);

            detect_key_events(key, raw_state, now);
            key->internal.current_state = raw_state;
        }

        // 检测长按和连发（仅当按键按下时）
        if (key->internal.current_state == KEY_STATE_PRESSED) {
            uint64_t press_duration = now - key->internal.press_timestamp;
            uint64_t long_press_ns = ms_to_ns(key->internal.long_press_ms);
            uint64_t repeat_ns = ms_to_ns(key->internal.repeat_ms);

            // 长按检测
            if (!key->internal.long_press_detected &&
                press_duration > long_press_ns) {
                key_log_info("GPIO%d 长按触发，持续时间: %lums\n",
                    key->internal.gpio_num,
                    press_duration / 1000000);
                fire_key_event(key, KEY_EV_LONG_PRESS);
                key->internal.long_press_detected = 1;
                key->internal.hold_repeat_count = 0;
            }

            // 连发检测
            if (key->internal.long_press_detected) {
                uint64_t next_repeat_time = long_press_ns +
                    ((key->internal.hold_repeat_count + 1) * repeat_ns);

                if (press_duration > next_repeat_time) {
                    key_log_info("GPIO%d 连发 #%d，持续时间: %lums\n",
                        key->internal.gpio_num,
                        key->internal.hold_repeat_count + 1,
                        press_duration / 1000000);
                    fire_key_event(key, KEY_EV_HOLD_REPEAT);
                    key->internal.hold_repeat_count++;
                }
            }
        }
    }

    // 双击超时检测（无论按键状态如何）
    if (key->internal.click_count > 0) {
        uint64_t double_click_ns = ms_to_ns(key->internal.double_click_ms);
        uint64_t time_since_first_click = now - key->internal.first_click_time;

        if (time_since_first_click > double_click_ns) {
            if (key->internal.click_count == 1) {
                // 单击事件
                key_log_info("GPIO%d 单击超时触发\n", key->internal.gpio_num);
                fire_key_event(key, KEY_EV_CLICK);
            } else {
                key_log_info("GPIO%d 双击超时取消 (click_count=%d)\n",
                    key->internal.gpio_num,
                    key->internal.click_count);
            }
            key->internal.click_count = 0;
        }
    }
}

/**
 * @brief 检测按键事件
 */
static void detect_key_events(struct key_handle *key, int new_state, uint64_t event_time)
{
    if (new_state == KEY_STATE_PRESSED) {
        // 按下事件
        key_log_info("GPIO%d 检测到按下\n", key->internal.gpio_num);
        fire_key_event(key, KEY_EV_PRESSED);
        key->internal.press_timestamp = event_time;
        key->internal.long_press_detected = 0;
        key->internal.hold_repeat_count = 0;

    } else if (new_state == KEY_STATE_RELEASED) {
        // 释放事件
        key_log_info("GPIO%d 检测到释放\n", key->internal.gpio_num);
        fire_key_event(key, KEY_EV_RELEASED);
        key->internal.release_timestamp = event_time;

        if (key->internal.long_press_detected) {
            key_log_info("GPIO%d 长按释放，取消单击/双击检测\n", key->internal.gpio_num);
            key->internal.click_count = 0;
            key->internal.first_click_time = 0;
            key->internal.long_press_detected = 0;
            key->internal.hold_repeat_count = 0;
            return;
        }

        // 检测双击
        uint64_t double_click_ns = ms_to_ns(key->internal.double_click_ms);

        if (key->internal.click_count == 0) {
            // 第一次点击
            key->internal.click_count = 1;
            key->internal.first_click_time = event_time;
            key_log_info("GPIO%d 第一次点击记录\n", key->internal.gpio_num);

        } else {
            // 检查是否在双击时间窗口内
            uint64_t time_since_first_click = event_time - key->internal.first_click_time;

            if (time_since_first_click <= double_click_ns) {
                // 双击事件
                key_log_info("GPIO%d 双击检测成功，时间间隔: %lums\n",
                    key->internal.gpio_num,
                    time_since_first_click / 1000000);
                fire_key_event(key, KEY_EV_DOUBLE_CLICK);
                key->internal.click_count = 0;
            } else {
                // 超时，触发之前的单击，开始新的计数
                key_log_info("GPIO%d 双击超时，触发之前的单击\n", key->internal.gpio_num);
                fire_key_event(key, KEY_EV_CLICK);
                key->internal.click_count = 1;
                key->internal.first_click_time = event_time;
            }
        }
    }

    // 更新最后稳定状态（已经在process_key_event中更新）
}

/**
 * @brief 触发按键事件回调
 */
static void fire_key_event(struct key_handle *key, key_event_t event)
{
    if (!key || !key->internal.callback) {
        return;
    }

#ifdef KEY_DEBUG
    // 打印事件信息（调试用）
    const char *event_names[] = {
        "按下", "释放", "单击", "双击", "长按", "连发"
    };

    printf("key:%p, GPIO%d: %s\n", key, key->internal.gpio_num, event_names[event]);
#endif

    // 调用用户回调
    key->internal.callback(key, event, key->internal.user_data);
}

/**
 * @brief 读取GPIO状态
 */
static int read_gpio_state(struct key_handle *key)
{
#if defined(LIBGPIOD_V2)
    if (!key || !key->req) {
        return -1;
    }

    enum gpiod_line_value v = gpiod_line_request_get_value(key->req, key->offset);
    if ((int)v < 0) {
        return -1;
    }

    int value = (v == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
#else
    if (!key || !key->line) {
        return -1;
    }

    int value = gpiod_line_get_value(key->line);
    if (value < 0) {
        return -1;
    }
#endif

    // 根据有效电平转换为逻辑状态
    if (key->internal.active_low) {
        return (value == 0) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
    } else {
        return (value == 1) ? KEY_STATE_PRESSED : KEY_STATE_RELEASED;
    }
}

/**
 * @brief 打开GPIO芯片
 */
#if defined(LIBGPIOD_V2)
static struct gpiod_chip* open_gpio_chip(int gpio_num, unsigned int *offset)
{
    if (gpio_num <= 0 || !offset) {
        return NULL;
    }
    char chip_name[20] = {0};
    for (int i = 0; i < 4; i++) {
        if (gpio_num >= i * 32 && gpio_num < (i + 1) * 32) {
            *offset = gpio_num - i * 32;
            snprintf(chip_name, sizeof(chip_name), "/dev/gpiochip%d", i);
            break;
        }
    }
    if (chip_name[0] == '\0') {
        return NULL;
    }
    printf("gpio_index=%d, offset=%u, chip_name=%s\n", gpio_num, *offset, chip_name);
    return gpiod_chip_open(chip_name);
}
#else
static struct gpiod_chip* open_gpio_chip(int gpio_num)
{
    if (gpio_num <= 0) {
        return NULL;
    }
    // 这里简化处理，实际应该根据GPIO编号确定芯片
    // 例如：树莓派通常是 "gpiochip0"
    struct gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        chip = gpiod_chip_open_by_name("gpiochip1");
    }
    return chip;
}
#endif

/**
 * @brief 获取当前时间戳（纳秒）
 */
static uint64_t get_timestamp_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/**
 * @brief 毫秒转纳秒
 */
static int ms_to_ns(int ms)
{
    return ms * 1000000;
}
