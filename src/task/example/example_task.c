/*
* Change Logs:
* Date            Author          Notes
* 2023-09-05      ChuShicheng     first version
*/

#include "example_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include <stm32f4xx.h>
/* C板预留的 uart6 */
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

static rt_device_t serial = RT_NULL;
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
static uint8_t rx_buffer[64];
static rt_size_t rx_length = 0;

int8_t flag = 0;
static uint16_t cx[2],cy[2];
static uint16_t cx_output,cy_output;
float cx_a = 0.93;
float cy_b = 0.93;
uint16_t x_ref = 160,y_ref = 120;


float Angle_Pitch= -60;
float Angle_Yaw=0;

static struct openmv_controller_t{
    pid_obj_t *cx_pid;
    pid_obj_t *cy_pid;
}openmv_controller;


static struct chassis_controller_t{
    pid_obj_t *speed_pid;
}chassis_controller;

static struct gimbal_controller_t{
    pid_obj_t *speed_pid;
    pid_obj_t *angle_pid_p;
    pid_obj_t *angle_pid_y;

}gimbal_controlelr;

static dji_motor_object_t *chassis_motor;
static dji_motor_object_t *gimbal_motor_p;
static dji_motor_object_t *gimbal_motor_y;


static rt_int16_t chassis_control(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(chassis_controller.speed_pid, measure.speed_rpm, 1000);
    return set;
}

static rt_int16_t gimbal_control_p(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = -pid_calculate(gimbal_controlelr.angle_pid_p, measure.total_angle, Angle_Pitch);
    return set;
}

static rt_int16_t gimbal_control_y(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = -pid_calculate(gimbal_controlelr.angle_pid_y, measure.total_angle, Angle_Yaw);
    return set;
}

static void example_init()
{
    pid_config_t chassis_speed_config = {
            .Kp = 10, // 4.5
            .Ki = 0,  // 0
            .Kd = 0,  // 0
            .IntegralLimit = 3000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 12000,
    };
    pid_config_t gimbal_angle_config_p = {
            .Kp = 15,  // 50
            .Ki = 7.5, // 200
            .Kd = 0,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 3000,
            .MaxOut = 20000,
    };

    pid_config_t gimbal_angle_config_y = {
            .Kp = 15,  // 50
            .Ki = 7.5, // 200
            .Kd = 0,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .IntegralLimit = 3000,
            .MaxOut = 20000,
    };
    pid_config_t openmv_config_cx = {
        .Kp = 0.3,
        .Ki = 0.25,
        .Kd = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 120,
        .MaxOut = 120,
    };
    pid_config_t openmv_config_cy = {
        .Kp = 0.3,
        .Ki = 0.25,
        .Kd = 0,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit = 120,
        .MaxOut = 120,
    };

    chassis_controller.speed_pid = pid_register(&chassis_speed_config);
    gimbal_controlelr.angle_pid_p = pid_register(&gimbal_angle_config_p);
    gimbal_controlelr.angle_pid_y = pid_register(&gimbal_angle_config_y);

    openmv_controller.cx_pid = pid_register(&openmv_config_cx);
    openmv_controller.cy_pid = pid_register(&openmv_config_cy);

    motor_config_t chassis_motor_config = {
            .motor_type = M3508,
            .can_name = CAN_CHASSIS,
            .rx_id = 0x201,
            .controller = &chassis_controller,
    };
    motor_config_t gimbal_motor_config_p = {
            .motor_type = GM6020,
            .can_name = CAN_CHASSIS,
            .rx_id = 0x207,
            .controller = &gimbal_controlelr,
    };

    motor_config_t gimbal_motor_config_y = {
            .motor_type = GM6020,
            .can_name = CAN_CHASSIS,
            .rx_id = 0x208,
            .controller = &gimbal_controlelr,
    };
    //chassis_motor = dji_motor_register(&chassis_motor_config, chassis_control);
    gimbal_motor_p = dji_motor_register(&gimbal_motor_config_p, gimbal_control_p);
    gimbal_motor_y = dji_motor_register(&gimbal_motor_config_y, gimbal_control_y);

}


void example_thread_entry(void *argument)
{

    static float example_dt;
    static float example_start;
    example_init();
    /* step1：查找名为 "vcom" 的虚拟串口设备*/
    serial = rt_device_find("uart6");
    if (!serial)
    {
        LOG_E("Find UART device failed!\n");
    }

    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_115200;        //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 128;                   //修改缓冲区 buff size 为 128
    config.parity    = PARITY_NONE;           //无奇偶校验位

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
    /* step4：打开串口设备。以中断接收及轮询发送模式打开串口设备*/
    if (serial)
        rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, serial_input);
    static float cx_angle_yaw, cy_angle_pitch;
    LOG_I("Example Task Start");
    for (;;)
    {
        example_start = dwt_get_time_ms();
        /* 这里可以添加任务处理代码 */
        if(flag == 2)
        {
            flag = 0;
            cx_angle_yaw =pid_calculate(openmv_controller.cx_pid, cx_output, x_ref);
            cy_angle_pitch =pid_calculate(openmv_controller.cy_pid, cy_output, y_ref);


            //保证pitch轴正向偏转不大于40度，防止扯线
            if(cy_angle_pitch > 40)
            {
                cy_angle_pitch = 40;
            }
            //其余情况通过pid输出限幅进行控制（不超过120度）

                Angle_Pitch = cy_angle_pitch;
                Angle_Yaw = cx_angle_yaw;
        }



        rt_thread_delay(10);
        /* 用于调试监测线程调度使用 */
        example_dt = dwt_get_time_ms() - example_start;
        if (example_dt > 1)
            LOG_E("Example Task is being DELAY! dt = [%f]", &example_dt);

        rt_thread_delay(1);
    }
}
// 数据解析函数
void parse_uart_data(uint8_t *data, rt_size_t length)
{
    if (length < 6 || data[0] != 0x2C || data[length - 1] != 0x5B)
    {
        // 数据包不完整或格式错误
        rt_kprintf("Invalid data packet\n");
        flag = 1;
        return;
    }
    cx[1] = cx_output;
    cy[1] = cy_output;
    cx[0] = (data[2]<<8) | data[3];//cx范围超过了255，用两个字节接收
    cy[0] = data[4];
    flag = 2;//表示收到了坐标
    /*一阶低通滤波*/
    cx_output = cx_a*cx[0] + (1-cx_a)*cx[1];
    cy_output = cy_b*cy[0] + (1-cy_b)*cy[1];
    rt_kprintf("Parsed cx: %d, cy: %d\n", cx, cy);
}
static rt_err_t serial_input(rt_device_t dev, rt_size_t size)
{
    rt_size_t len;
    // 读取接收到的数据到缓冲区
    len = rt_device_read(dev, 0, rx_buffer + rx_length, size);
    if (len > 0)
    {
        rx_length += len;
        // 检查是否收到了完整的数据包（这里假设数据包以0x5B结束）
        if (rx_length >= 6 && rx_buffer[rx_length - 1] == 0x5B)
        {
            // 调用数据解析函数
            parse_uart_data(rx_buffer, rx_length);
            // 重置接收长度
            rx_length = 0;
        }
    }
    LOG_D("%d %d\n", cx, cy);
    return RT_EOK;
}