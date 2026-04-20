/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <key.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

/* --- 全局变量 --- */
static volatile int running = 1;

/* --- 信号处理 --- */
void signal_handler(int sig)
{
    if (sig == SIGINT) {
        printf("\n收到 Ctrl+C，正在退出...\n");
        running = 0;
    }
}

/* --- 按键回调函数 --- */
void key_event_callback(struct key_handle *key, key_event_t event, void *user_data)
{
    const char *key_name = (const char *)user_data;
    static int click_count = 0;
    static int double_click_count = 0;
    static int long_press_count = 0;

    if (!key) {
        return;
    }

    switch (event) {
        case KEY_EV_PRESSED:
            printf("[%s] 物理按下\n", key_name);
            break;

        case KEY_EV_RELEASED:
            printf("[%s] 物理释放\n", key_name);
            break;

        case KEY_EV_CLICK:
            click_count++;
            printf("[%s] 单击事件 (#%d)\n", key_name, click_count);
            // 这里可以执行单击操作，如切换状态
            break;

        case KEY_EV_DOUBLE_CLICK:
            double_click_count++;
            printf("[%s] 双击事件 (#%d)\n", key_name, double_click_count);
            // 这里可以执行双击操作，如打开设置
            break;

        case KEY_EV_LONG_PRESS:
            long_press_count++;
            printf("[%s] 长按事件 (#%d)\n", key_name, long_press_count);
            // 这里可以执行长按操作，如重置设备
            break;

        case KEY_EV_HOLD_REPEAT:
            printf("[%s] 连发事件\n", key_name);
            // 这里可以执行连发操作，如音量连续调节
            break;

        default:
            printf("[%s] 未知事件: %d\n", key_name, event);
            break;
    }
}

/* --- 帮助信息 --- */
void print_help(void)
{
    printf("\n");
    printf("==============================\n");
    printf("    按键驱动演示程序\n");
    printf("==============================\n");
    printf("\n");
    printf("可用的按键操作:\n");
    printf("  1. 短按一次        -> 单击事件\n");
    printf("  2. 快速按两次      -> 双击事件\n");
    printf("  3. 按住1.5秒以上   -> 长按事件\n");
    printf("  4. 长按后保持      -> 连发事件\n");
    printf("\n");
    printf("按 Ctrl+C 退出程序\n");
    printf("\n");
}

/* --- 主函数 --- */
int main(void)
{
    printf("启动按键驱动演示程序...\n");

    // 设置信号处理
    signal(SIGINT, signal_handler);

    // 打印帮助信息
    print_help();

    // 启动按键服务
    if (key_service_start() != 0) {
        fprintf(stderr, "启动按键服务失败\n");
        return 1;
    }

    // 配置按键1（假设连接到 GPIO17，低电平有效）
    key_config_t key1_config = {
        .gpio_num = 113,
        .active_low = 0,           // 低电平有效
        .long_press_ms = 1500,     // 1.5秒长按
        .double_click_ms = 300     // 300ms内双击有效
    };

    // 配置按键2（假设连接到 GPIO18，高电平有效）
    key_config_t key2_config = {
        .gpio_num = 114,
        .active_low = 0,           // 高电平有效
        .long_press_ms = 2000,     // 2秒长按
        .double_click_ms = 400     // 400ms内双击有效
    };

    // 添加按键1
    struct key_handle *key1 = key_add_gpio(&key1_config, key_event_callback, "按键1");
    if (!key1) {
        fprintf(stderr, "添加按键1失败\n");
        key_service_stop();
        return 1;
    }

    // 添加按键2
    struct key_handle *key2 = key_add_gpio(&key2_config, key_event_callback, "按键2");
    if (!key2) {
        fprintf(stderr, "添加按键2失败\n");
        key_remove(key1);
        key_service_stop();
        return 1;
    }

    printf("按键已添加:\n");
    printf("  按键1: GPIO%d (%s有效)\n",
        key1_config.gpio_num,
        key1_config.active_low ? "低电平" : "高电平");
    printf("  按键2: GPIO%d (%s有效)\n",
        key2_config.gpio_num,
        key2_config.active_low ? "低电平" : "高电平");

    printf("\n等待按键事件...\n\n");

    // 主循环
    int count = 0;
    while (running) {
        sleep(1);
        count++;

        // 每10秒打印一次状态
        if (count % 10 == 0) {
            printf("程序运行中... (%d 秒)\n", count);
        }
    }

    // 清理资源
    printf("\n清理资源...\n");

    // 删除按键
    if (key1) {
        key_remove(key1);
        printf("按键1已删除\n");
    }

    if (key2) {
        key_remove(key2);
        printf("按键2已删除\n");
    }

    // 停止服务
    key_service_stop();

    printf("程序退出\n");
    return 0;
}
