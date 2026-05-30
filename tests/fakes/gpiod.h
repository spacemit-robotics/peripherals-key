/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef GPIOD_H
#define GPIOD_H

struct gpiod_chip;
struct gpiod_line;

struct gpiod_line_request_config {
    const char *consumer;
    int request_type;
    int flags;
};

#define GPIOD_LINE_REQUEST_DIRECTION_INPUT 1
#define GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP 0x01
#define GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_DOWN 0x02

void fake_gpiod_reset(void);
void fake_gpiod_set_value(int offset, int value);
int fake_gpiod_get_release_count(void);

struct gpiod_chip *gpiod_chip_open_by_name(const char *name);
void gpiod_chip_close(struct gpiod_chip *chip);
struct gpiod_line *gpiod_chip_get_line(struct gpiod_chip *chip, unsigned int offset);
int gpiod_line_request(struct gpiod_line *line,
    const struct gpiod_line_request_config *config, int default_val);
int gpiod_line_get_value(struct gpiod_line *line);
void gpiod_line_release(struct gpiod_line *line);

#endif /* GPIOD_H */
