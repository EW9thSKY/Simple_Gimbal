//
// Created by 14685 on 2023/9/5.
//

#ifndef RTTHREAD_EXAMPLE_TASK_H
#define RTTHREAD_EXAMPLE_TASK_H

#include <rtthread.h>

void example_thread_entry(void *argument);
void parse_uart_data(uint8_t *data, rt_size_t length);
static rt_err_t serial_input(rt_device_t dev, rt_size_t size);
#endif //RTTHREAD_EXAMPLE_TASK_H
