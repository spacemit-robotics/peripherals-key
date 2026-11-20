# Key 按键组件

## 项目简介
面向 Linux GPIO 的按键驱动组件，提供统一的按键事件识别与回调机制，解决按键去抖、单双击、长按与连发等常见交互逻辑的重复实现问题。

## 功能特性
支持：
- 基于 libgpiod 的 GPIO 按键输入
- 按下、释放、单击、双击、长按、长按连发事件
- 可配置低电平/高电平有效、长按与双击时间阈值

不支持：
- 矩阵键盘、ADC 按键或编码器类输入
- 运行时动态调整连发间隔（当前固定 200ms）
- 非 Linux/非 libgpiod 环境

## 快速开始
最短路径跑起来：编译运行 `./test_key` 并运行查看事件打印。

### 环境准备
- Linux 系统，支持 libgpiod（v1/v2 均可）
- `gcc`/`cmake`/`make` 构建工具
- 确保当前用户具备访问 `/dev/gpiochip*` 的权限（必要时使用 `sudo`）

### 构建编译
以下为脱离 SDK 的独立构建方式：
```bash
mkdir -p build
cd build
cmake ..
make -j
```

### 运行示例
```bash
sudo ./build/test_key
```

示例程序也支持从命令行指定两个 GPIO 和有效电平：
```bash
./build/test_key [gpio1 gpio2 [active_low]]
```

- 默认使用 GPIO113、GPIO114，`active_low=0`（高电平有效）。
- `active_low=1` 表示低电平有效，两个按键共用此设置。
- MUSE Pi Pro 上 GPIO49、GPIO50 接地触发时，可以运行：
  ```bash
  sudo ./build/test_key 49 50 1
  ```
- 可以运行 `./build/test_key --help` 查看用法。GPIO 编号必须是非负整数，`active_low` 只能是 `0` 或 `1`。

### 代码示例
```c
#include <key.h>
#include <stdio.h>

static void on_key_event(struct key_handle *key, key_event_t event, void *user_data)
{
    const char *name = (const char *)user_data;
    printf("[%s] event=%d\n", name, event);
}

int main(void)
{
    key_service_start();

    key_config_t cfg = {
        .gpio_num = 83,
        .active_low = 1,
        .long_press_ms = 1500,
        .double_click_ms = 300
    };

    struct key_handle *h = key_add_gpio(&cfg, on_key_event, "KEY1");
    if (!h) {
        return 1;
    }

    pause(); /* 等待事件 */
    return 0;
}
```

## 详细使用
保留，引用到后续的官方文档。

## 常见问题
- 程序无事件输出：检查 GPIO 编号是否为 Linux 逻辑编号，并确认 `active_low` 配置是否正确。
- 无法打开 GPIO：确认 `/dev/gpiochip*` 权限，建议加入 gpio 组或以 `sudo` 运行。
- 双击/长按不灵敏：根据硬件手感调整 `double_click_ms` 与 `long_press_ms`。

## 版本与发布

版本以本目录 `package.xml` 中的 `<version>` 为准。

| 版本   | 日期       | 说明 |
| ------ | ---------- | ---- |
| 0.1.0  | 2026-02-28 | 初始版本，支持 libgpiod v1/v2 的 GPIO 按键事件识别。 |

## 贡献方式

欢迎参与贡献：提交 Issue 反馈问题，或通过 Pull Request 提交代码。

- **编码规范**：本组件 C 代码遵循 [Google C++ 风格指南](https://google.github.io/styleguide/cppguide.html)（C 相关部分），请按该规范编写与修改代码。
- **提交前检查**：请在提交前运行本仓库的 lint 脚本，确保通过风格检查：
  ```bash
  # 在仓库根目录执行（检查全仓库）
  bash scripts/lint/lint_cpp.sh

  # 仅检查本组件
  bash scripts/lint/lint_cpp.sh components/peripherals/key
  ```
  脚本路径：`scripts/lint/lint_cpp.sh`。若未安装 `cpplint`，可先执行：`pip install cpplint` 或 `pipx install cpplint`。
- **提交说明**：提交 Issue 或 PR 前请描述硬件型号、GPIO 编号与复现步骤。

## License
本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
