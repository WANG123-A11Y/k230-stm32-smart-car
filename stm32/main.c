/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 增量式 PID（车轮电机调速用，可多实例） */
typedef struct {
    float Kp, Ki, Kd;                  /* PID 系数 */
    float Err, Err_Last, Err_Prev;     /* 当前/上一次/上两次误差 */
    float Out_Inc;                     /* 增量输出 */
    float Out;                         /* 当前输出（PWM 比较值，允许为负表示反转） */
    float PWM_Max, PWM_Min;            /* 输出限幅 */
} PID_MotorTypeDef;

/* 单个轮子的方向引脚（N1/N2 为电机驱动芯片方向脚） */
typedef struct {
    GPIO_TypeDef *n1_port; uint16_t n1_pin;
    GPIO_TypeDef *n2_port; uint16_t n2_pin;
} WheelGpioTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 编码器参数：CubeMX 中 TIM2~5 为 TIM_ENCODERMODE_TI1（2 倍频计数），
 * 每圈计数 = 线数 x2；若改成 TI12（4 倍频），改为 x4 */
#define ENCODER_LINE      330
#define ENCODER_PPR       (ENCODER_LINE * 2)   /* 每圈脉冲数 */
#define WHEEL_SAMPLE_MS   100U                 /* 测速与控制周期 */
#define WHEEL_SELFTEST_ENABLE 1                /* 启动自检开关：验证完成后改 0 关闭 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static PID_MotorTypeDef wheel_pid[4];       /* 4 个轮子各一个 PID 实例 */
static float            wheel_target_rpm[4];/* 各轮目标转速(RPM，带符号)，0=未启用闭环 */
static uint32_t         last_enc_cnt[4];    /* 上次编码器计数 */
static uint32_t         last_enc_ms;        /* 上次测速时刻 */
static uint32_t         wheel_next_ms;      /* 下次闭环控制时刻 */
static const uint32_t wheel_ch[4] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};

/* 按键控制相关（放在 USER CODE 块内，防止 CubeMX 重新生成时丢失） */
uint8_t  run_flag = 0;
uint8_t  speed_level = 0;
uint16_t speed_table[] = {20, 40, 60, 80};
#define  LEVEL_MAX 3
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 100);
        return ch;
    }
void wheel_init(void){
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

/* 按键控制（非阻塞，边沿触发）
 *   K0 = PB9 ：启停切换（run_flag 取反）
 *   K1 = PB8 ：调速档位切换（speed_level 0~LEVEL_MAX 循环）
 * 接线：按键一端接 GND，另一端接 PB8/PB9；芯片内部上拉，按下为低电平。
 * 说明：只在"按下"的下降沿触发一次，长按不会连续触发；用 HAL_GetTick 消抖，
 *       不阻塞主循环，云台跟踪与 PID 调速不受影响。
 */
void key_scan(void)
{
    static uint32_t k0_last_ms = 0, k1_last_ms = 0;
    static uint8_t  k0_last = 1, k1_last = 1;   /* 1=松开(高) 0=按下(低) */
    uint32_t now = HAL_GetTick();
    uint8_t k0 = HAL_GPIO_ReadPin(K0_GPIO_Port, K0_Pin);
    uint8_t k1 = HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin);

    /* K0 下降沿：启停切换 */
    if (k0 == 0 && k0_last == 1 && (now - k0_last_ms) > 20)
    {
        run_flag = !run_flag;
        k0_last_ms = now;
        if (run_flag)
            printf("RUN, speed=%d RPM\r\n", (int)speed_table[speed_level]);
        else
            printf("STOP\r\n");
    }

    /* K1 下降沿：调速档位循环（运行/停止状态下都可切换） */
    if (k1 == 0 && k1_last == 1 && (now - k1_last_ms) > 20)
    {
        speed_level++;
        if (speed_level > LEVEL_MAX) speed_level = 0;
        k1_last_ms = now;
        printf("SPEED LEVEL %d = %d RPM\r\n", (int)speed_level, (int)speed_table[speed_level]);
    }

    k0_last = k0;
    k1_last = k1;
}

/* ============================================================================
//  * 车轮电机闭环调速（编码器测速 + 增量式 PID）
//  * 用法：
//  *   1) 启动时调用 encoder_init()、wheel_pid_init()；
//  *   2) 主循环里调用 wheel_speed_task()（内部按 WHEEL_SAMPLE_MS 节拍自调度，非阻塞）；
//  *   3) 上层运动接口（car_forward/car_backward/car_turn_*/
//  *      通过 wheel_speed_set() 设定目标转速，全部走 PID 闭环
//  * ==========================================================================*/

/* 轮子 i 对应的 TIM8 通道与方向引脚（与 car_forward/car_backward 引脚约定一致） */

static const WheelGpioTypeDef wheel_gpio[4] = {
    {P1N1_GPIO_Port, P1N1_Pin, P1N2_GPIO_Port, P1N2_Pin},
    {P2N2_GPIO_Port, P2N2_Pin, P2N1_GPIO_Port, P2N1_Pin},
    {P3N1_GPIO_Port, P3N1_Pin, P3N2_GPIO_Port, P3N2_Pin},
    {P4N2_GPIO_Port, P4N2_Pin, P4N1_GPIO_Port, P4N1_Pin},
};

/* 编码器方向修正：自检阶段自动校准后写入，无需手动改（-1 表示反向） */
static int enc_dir_sign[4] = {1, 1, 1, 1};

static float wheel_abs(float v) { return (v < 0.0f) ? -v : v; }

/* ---------- PID ---------- */
void Motor_PID_Init(PID_MotorTypeDef *pid, TIM_HandleTypeDef *htim)
{
    pid->Kp = 0.5f;   /* 保守初值：若收敛慢再逐步加大，若震荡则减小 */
    pid->Ki = 0.1f;
    pid->Kd = 0.0f;   /* 先不加微分，避免 100ms 采样下放大测速噪声 */
    pid->Err = 0.0f;
    pid->Err_Last = 0.0f;
    pid->Err_Prev = 0.0f;
    pid->Out_Inc = 0.0f;
    pid->Out = 0.0f;
    /* 输出允许为负：负值表示反转（方向引脚翻转），故下限 = -ARR */
    pid->PWM_Max = (float)htim->Init.Period;   /* TIM8: 99 */
    pid->PWM_Min = -(float)htim->Init.Period;
}

float Motor_PID_Calc(PID_MotorTypeDef *pid, float set_spd, float cur_spd)
{
    float err = set_spd - cur_spd;

    /* 增量式 PID（对输出限幅天然抗积分饱和） */
    pid->Out_Inc = pid->Kp * (err - pid->Err_Last)
                 + pid->Ki * err
                 + pid->Kd * (err - 2.0f * pid->Err_Last + pid->Err_Prev);

    pid->Out += pid->Out_Inc;
    if (pid->Out > pid->PWM_Max) pid->Out = pid->PWM_Max;
    if (pid->Out < pid->PWM_Min) pid->Out = pid->PWM_Min;

    pid->Err_Prev = pid->Err_Last;
    pid->Err_Last = err;

    return pid->Out;
}

/* ---------- 测速 ---------- */
static TIM_HandleTypeDef *enc_htim[4] = {&htim2, &htim3, &htim4, &htim5};

/* 编码器启动 + 测速基准初始化（必须调用，否则编码器不计数！） */
void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

    for (int i = 0; i < 4; i++)
        last_enc_cnt[i] = __HAL_TIM_GET_COUNTER(enc_htim[i]);
    last_enc_ms   = HAL_GetTick();
    wheel_next_ms = last_enc_ms + WHEEL_SAMPLE_MS;
}

/* 读取 4 轮转速(RPM，带符号)。按实际经过时间换算，
 * 调用周期不必严格等于 100ms；计数器回绕由无符号减法自动处理 */
void get_wheel_rpm(float rpm_buf[4])
{
    uint32_t now   = HAL_GetTick();
    uint32_t dt_ms = now - last_enc_ms;
    last_enc_ms    = now;
    if (dt_ms == 0) return;   /* 距上次不足 1ms，保持上次结果 */

    for (int i = 0; i < 4; i++)
    {
        uint32_t curr = __HAL_TIM_GET_COUNTER(enc_htim[i]);
        int32_t diff  = (int32_t)(curr - last_enc_cnt[i]);   /* 无符号减法自动处理回绕 */
        last_enc_cnt[i] = curr;
        /* RPM = 计数差 / 每圈计数 / (dt_ms / 60000) */
        rpm_buf[i] = (float)diff * enc_dir_sign[i] * 60000.0f
                   / ((float)ENCODER_PPR * (float)dt_ms);
    }
}

/* ---------- 闭环控制 ---------- */
void wheel_pid_init(void)
{
    for (int i = 0; i < 4; i++)
    {
        Motor_PID_Init(&wheel_pid[i], &htim8);
        wheel_target_rpm[i] = 0.0f;
    }
}

/* 设定目标转速(RPM，带符号)；传 0 = 退出闭环，滑行停车 */
void wheel_speed_set(int i, float rpm)
{
    if (i < 0 || i > 3) return;

    if (rpm == 0.0f)
    {
        /* 退出闭环：滑行（N1=N2=LOW），PWM 清零 */
        wheel_target_rpm[i] = 0.0f;
        wheel_pid[i].Out = 0.0f;
        __HAL_TIM_SET_COMPARE(&htim8, wheel_ch[i], 0);
        HAL_GPIO_WritePin(wheel_gpio[i].n1_port, wheel_gpio[i].n1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(wheel_gpio[i].n2_port, wheel_gpio[i].n2_pin, GPIO_PIN_RESET);
    }
    else if (wheel_target_rpm[i] == 0.0f)
    {
        /* 开环 -> 闭环切换：PID 输出从当前 PWM 起步，避免突跳 */
        wheel_target_rpm[i] = rpm;
        wheel_pid[i].Out = (float)__HAL_TIM_GET_COMPARE(&htim8, wheel_ch[i]);
    }
    else
    {
        wheel_target_rpm[i] = rpm;
    }
}

/* 对单个轮子执行一次闭环（PID 计算 + 方向 + PWM 输出），返回有符号输出值 */
static float wheel_pid_apply(int i, float cur_rpm)
{
    float out = Motor_PID_Calc(&wheel_pid[i], wheel_target_rpm[i], cur_rpm);
    uint16_t duty = (uint16_t)wheel_abs(out);

    /* 方向（与 car_forward/car_backward 约定一致）：
     * 正转 N1=H/N2=L；反转 N1=L/N2=H */
    HAL_GPIO_WritePin(wheel_gpio[i].n1_port, wheel_gpio[i].n1_pin,
                      (out >= 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(wheel_gpio[i].n2_port, wheel_gpio[i].n2_pin,
                      (out >= 0.0f) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim8, wheel_ch[i], duty);
    return out;
}

/* 编码器故障检测：目标非零、PWM 已饱和、但转速长期接近 0 → 编码器无反馈（失速） */
static void wheel_encoder_fault_check(const float rpm[4])
{
    static uint8_t fault_cnt[4] = {0};
    for (int i = 0; i < 4; i++)
    {
        if (wheel_target_rpm[i] != 0.0f &&
            wheel_abs(rpm[i]) < 3.0f &&
            wheel_abs(wheel_pid[i].Out) >= 95.0f)
        {
            if (++fault_cnt[i] >= 20)   /* 连续 20 个周期(约 2s)判为故障 */
            {
                fault_cnt[i] = 0;
                printf("WARN: wheel%d encoder no feedback (stalled)! Check encoder A/B wiring\r\n", i + 1);
            }
        }
        else
        {
            fault_cnt[i] = 0;
        }
    }
}

/* 主循环调用：按 WHEEL_SAMPLE_MS 节拍测速并闭环调速（非阻塞） */
void wheel_speed_task(void)
{
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - wheel_next_ms) < 0) return;
    wheel_next_ms = now + WHEEL_SAMPLE_MS;

    float rpm[4];
    get_wheel_rpm(rpm);

    for (int i = 0; i < 4; i++)
    {
        if (wheel_target_rpm[i] == 0.0f) continue;
        wheel_pid_apply(i, rpm[i]);
    }

    wheel_encoder_fault_check(rpm);
}

/* ============================================================================
 * 基础运动接口（编码器增量式 PID 闭环实现，替代原开环 PWM 控制）
 *   car_set_speed(rpm)              ：全轮同速（带符号）
 *   car_forward / car_backward      ：闭环前进 / 后退
 *   car_turn_left / car_turn_right  ：原地自转（左转/右转，不前进）
 *   car_stop                        ：短路刹车（清目标 + PWM 0 + N1=N2=H）
 *   car_slip                        ：滑行（清目标 + PWM 0 + N1=N2=L）
 * ==========================================================================*/

#define RUN_RPM   60.0f   /* 前进/后退目标转速(RPM) */
#define TURN_RPM  25.0f   /* 原地自转转速(RPM) */

void car_set_speed(float rpm) {
    for (int i = 0; i < 4; i++) wheel_speed_set(i, rpm);
}

void car_forward(void)  { car_set_speed( RUN_RPM); }
void car_backward(void) { car_set_speed(-RUN_RPM); }

/* 原地自转（不前进）：左转 = 左轮(0,2)反转、右轮(1,3)正转；右转相反 */
void car_turn_left(void) {
    wheel_speed_set(0, -TURN_RPM);
    wheel_speed_set(1,  TURN_RPM);
    wheel_speed_set(2, -TURN_RPM);
    wheel_speed_set(3,  TURN_RPM);
}

void car_turn_right(void) {
    wheel_speed_set(0,  TURN_RPM);
    wheel_speed_set(1, -TURN_RPM);
    wheel_speed_set(2,  TURN_RPM);
    wheel_speed_set(3, -TURN_RPM);
}

/* 短路刹车 */
void car_stop(void) {
    for (int i = 0; i < 4; i++)
    {
        wheel_target_rpm[i] = 0.0f;
        wheel_pid[i].Out = 0.0f;
        __HAL_TIM_SET_COMPARE(&htim8, wheel_ch[i], 0);
        HAL_GPIO_WritePin(wheel_gpio[i].n1_port, wheel_gpio[i].n1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(wheel_gpio[i].n2_port, wheel_gpio[i].n2_pin, GPIO_PIN_SET);
    }
}

/* 滑行 */
void car_slip(void) {
    car_set_speed(0.0f);
}

/* ============================================================
 * 车轮电机自检（阻塞式，main 启动阶段调用，开关 WHEEL_SELFTEST_ENABLE）
 * 通过 USART1 输出结果：
 *   [1] 编码器方向校准：逐轮正转，测计数方向，自动修正 enc_dir_sign
 *   [2] PID 稳定性 + 测速：打印 目标/实测/PWM，观察收敛与稳定性
 * 注：本工程 newlib-nano 默认无浮点 printf，转速按 x10 整数打印
 *     （如 605 = 60.5 RPM），避免依赖 -u _printf_float 链接选项。
 * ============================================================*/

/* 单轮编码器方向检测：正转驱动，测 300ms 计数变化，返回带符号计数差 */
static int32_t wheel_encoder_dir_detect(int i)
{
    uint32_t cnt0, cnt1;

    /* 正转方向驱动（N1=H/N2=L），PWM=50 */
    HAL_GPIO_WritePin(wheel_gpio[i].n1_port, wheel_gpio[i].n1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(wheel_gpio[i].n2_port, wheel_gpio[i].n2_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim8, wheel_ch[i], 50);
    HAL_Delay(200);                       /* 等电机转起来 */
    cnt0 = __HAL_TIM_GET_COUNTER(enc_htim[i]);
    HAL_Delay(300);
    cnt1 = __HAL_TIM_GET_COUNTER(enc_htim[i]);

    /* 停车（滑行） */
    __HAL_TIM_SET_COMPARE(&htim8, wheel_ch[i], 0);
    HAL_GPIO_WritePin(wheel_gpio[i].n1_port, wheel_gpio[i].n1_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(wheel_gpio[i].n2_port, wheel_gpio[i].n2_pin, GPIO_PIN_RESET);

    return (int32_t)(cnt1 - cnt0);
}

void wheel_selftest(void)
{
    float rpm[4];

    printf("\r\n===== Wheel Self-test Start =====\r\n");

    /* ---- 阶段1：编码器方向自动校准 ---- */
    printf("[1] Encoder direction calibration (lift wheels!)...\r\n");
    for (int i = 0; i < 4; i++)
    {
        int32_t d = wheel_encoder_dir_detect(i);
        enc_dir_sign[i] = (d >= 0) ? 1 : -1;
        if (d == 0)
            printf("    wheel%d: [ENCODER NOT WORKING] diff 0, check encoder A/B wiring\r\n", i + 1);
        else
            printf("    wheel%d: diff %5d, dir %s\r\n", i + 1, (int)d,
                   enc_dir_sign[i] > 0 ? "OK" : "REVERSED(fixed)");
        HAL_Delay(300);
    }
    /* 重新建立测速基准 */
    for (int i = 0; i < 4; i++) last_enc_cnt[i] = __HAL_TIM_GET_COUNTER(enc_htim[i]);
    last_enc_ms = HAL_GetTick();

    /* ---- 阶段2：PID 闭环稳定性 + 测速 ---- */
    printf("[2] PID forward 60 RPM, 3s (speed RPM x10, PWM=compare)\r\n");
    printf("  t/ms  set    m1    m2    m3    m4   p1  p2  p3  p4\r\n");
    car_forward();
    uint32_t t0 = HAL_GetTick();
    for (int k = 0; k < 30; k++)
    {
        HAL_Delay(WHEEL_SAMPLE_MS);
        get_wheel_rpm(rpm);
        int p[4];
        for (int i = 0; i < 4; i++) p[i] = (int)wheel_pid_apply(i, rpm[i]);
        printf("%5lu %4d  %5d %5d %5d %5d   %3d %3d %3d %3d\r\n",
               (unsigned long)(HAL_GetTick() - t0), 600,
               (int)(rpm[0]*10), (int)(rpm[1]*10), (int)(rpm[2]*10), (int)(rpm[3]*10),
               p[0], p[1], p[2], p[3]);
    }

    printf("[2] reverse -60 RPM, 3s ...\r\n");
    car_backward();
    t0 = HAL_GetTick();
    for (int k = 0; k < 30; k++)
    {
        HAL_Delay(WHEEL_SAMPLE_MS);
        get_wheel_rpm(rpm);
        int p[4];
        for (int i = 0; i < 4; i++) p[i] = (int)wheel_pid_apply(i, rpm[i]);
        printf("%5lu %4d  %5d %5d %5d %5d   %3d %3d %3d %3d\r\n",
               (unsigned long)(HAL_GetTick() - t0), -600,
               (int)(rpm[0]*10), (int)(rpm[1]*10), (int)(rpm[2]*10), (int)(rpm[3]*10),
               p[0], p[1], p[2], p[3]);
    }

    car_stop();
    printf("[2] PID closed-loop test done (rpm should converge, no oscillation)\r\n");
    printf("===== Wheel Self-test End =====\r\n\r\n");
}

/* ============================================================================
 * 云台视觉伺服（K230 目标跟踪）
 * 架构：K230 --USART2--> STM32F407（本程序：解析坐标 + PID 分析 + 组帧下发）
 *                        --USART3--> C06B 云台板(STM32F103)（接收解析 + 舵机驱动）
 * 接线：K230  -> USART2 (PA2/PA3)
 *       C06B  <- USART3 (PB10=TX -> C06B RX，PB11=RX -> C06B TX)，两端共地
 * 协议：K230 发 "x,y\r\n"（相对画面中心，右/下为正）
 *       F407 发 6 字节二进制帧（0xAA 指令码 水平角 俯仰角 校验和 0xBB）
 * ==========================================================================*/

/* ---------- 可调参数（按实际整定） ---------- */
#define TARGET_X        0        /* 目标中心 x（相对坐标，恒为 0；gimbal_uart_feed 已减 IMG_CENTER） */
#define TARGET_Y        0        /* 目标中心 y（相对坐标，恒为 0） */
#define DEAD_ZONE       8.0f     /* 死区：误差 < 8 像素不再动作，防止抖动 */

/* K230 检测帧分辨率（须与 K230 端一致）：800x480 → 中心 (400,240)
 * K230 端 rgb888p_size=[800,480]，YOLO 输出 cx 范围 0~800、cy 范围 0~480 */
#define IMG_CENTER_X    400
#define IMG_CENTER_Y    240

/* 云台机械行程限制（0~270° 映射 500~2500us，与 C06B 板一致）
 * 水平(yaw)中点 135°，俯仰(pitch)中点 90°（实际机械中心） */
#define YAW_ANGLE_MIN    5.0f
#define YAW_ANGLE_MAX    255.0f
#define PITCH_ANGLE_MIN  9.0f
#define PITCH_ANGLE_MAX  170.0f
#define YAW_CENTER       135.0f   /* 水平中点 */
#define PITCH_CENTER     90.0f    /* 俯仰中点 */

/* 位置式 PI 控制 + 变增益 + 抗积分饱和：
 *   目标角度 = 中心 + 方向 × (变增益KP×像素偏差 + 积分项)
 * 变增益：误差大(物体离中心远)用大增益快速逼近，误差小用小增益精细微调，
 *         让"快"与"稳"兼得（KP_MIN~KP_MAX 之间按误差线性过渡）。
 * 积分项消除稳态误差(让物体真正居中)；积分限幅 INTEGRAL_MAX 抗饱和防过冲。 */
#define KP_MAX        0.09f    /* 大误差(>=ERR_LARGE)时的增益：度/像素 */
#define KP_MIN        0.06f    /* 小误差(<=ERR_SMALL)时的增益：度/像素 */
#define ERR_LARGE     150.0f   /* 像素误差，超过用 KP_MAX */
#define ERR_SMALL     25.0f    /* 像素误差，低于用 KP_MIN */
#define KI_PIXEL      0.02f    /* 积分增益：度/像素/帧。消除稳态误差 */
#define INTEGRAL_MAX  130.0f   /* 积分限幅(度)：必须覆盖整个行程(相对135°最大±130°)，否则偏离中心远的物体永远偏在一边 */
#define INTEGRAL_STEP 2.0f     /* 积分每帧最大增量(度)：=60°/s 与舵机速度匹配，防大误差时积分冲太快 */
#define ANGLE_SMOOTH  0.4f     /* 角度平滑：0~1，越小越稳(不抖)，越大越跟手(快) */

/* 方向：+1 表示"像素正向 → 角度正向"，-1 反向。
 * 像素 x 右为正 / y 下为正；角度 yaw、pitch 增大。
 * 实测：物体在右 → yaw 减小(右转) → YAW_DIR=-1；
 *       物体在下 → pitch 增大(低头) → PITCH_DIR=+1 */
#define YAW_DIR          (-1.0f)
#define PITCH_DIR        (1.0f)

/* ---------- 数据结构 ---------- */
typedef struct {
    float yaw;      /* 当前偏航角(度) */
    float pitch;    /* 当前俯仰角(度) */
    float i_yaw;    /* yaw 积分项(度)，消除稳态误差 */
    float i_pitch;  /* pitch 积分项(度) */
    float last_dx;  /* 上一帧 x 偏差，用于检测"越过中心"变号 */
    float last_dy;  /* 上一帧 y 偏差 */
} gimbal_t;

static gimbal_t gimbal;

/* ---------- 基础工具 ---------- */
static float fabs_(float v) { return (v < 0.0f) ? -v : v; }

static float clamp_(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 变增益：|err| 越大增益越大，KP_MIN~KP_MAX 之间线性过渡 */
static float gain_schedule(float err) {
    float m = fabs_(err);
    if (m >= ERR_LARGE) return KP_MAX;
    if (m <= ERR_SMALL) return KP_MIN;
    return KP_MIN + (KP_MAX - KP_MIN) * (m - ERR_SMALL) / (ERR_LARGE - ERR_SMALL);
}

/* ---------- F407 -> C06B 帧协议常量 ---------- */
#define FRAME_HEAD        0xAA    /* 帧头 */
#define FRAME_TAIL        0xBB    /* 帧尾 */
#define CMD_GIMBAL_ANGLE  0x01    /* 云台角度指令 */
#define CMD_GIMBAL_RESET  0x02    /* 复位回中指令 */

/* ---------- 下发指令到 C06B 云台板 ----------
 * 帧格式（6 字节）：
 *   帧头(1B)  指令码(1B)       水平角(1B)  俯仰角(1B)  校验和(1B)  帧尾(1B)
 *   0xAA      0x01 角度指令    hor_angle   pit_angle   SUM        0xBB
 *   0xAA      0x02 复位指令    0x00        0x00        SUM        0xBB
 * 校验和 SUM = (帧头 + 指令码 + 水平角 + 俯仰角) 低 8 位；
 * C06B 收到后重算校验，不一致则整帧丢弃。
 * 角度为整数字节（单位：度），C06B 负责解析并驱动舵机。
 */
void gimbal_send(float yaw, float pitch) {
    uint8_t hor = (uint8_t)(yaw   + 0.5f);    /* 四舍五入到整数度 */
    uint8_t pit = (uint8_t)(pitch + 0.5f);
    uint8_t sum = (FRAME_HEAD + CMD_GIMBAL_ANGLE + hor + pit) & 0xFF;
    uint8_t frame[6] = {FRAME_HEAD, CMD_GIMBAL_ANGLE, hor, pit, sum, FRAME_TAIL};
    HAL_UART_Transmit(&huart3, frame, 6, 100);
}

/* 复位指令：云台回到中位 */
void gimbal_reset(void) {
    uint8_t sum = FRAME_HEAD + CMD_GIMBAL_RESET + 0x00 + 0x00;
    uint8_t frame[6] = {FRAME_HEAD, CMD_GIMBAL_RESET, 0x00, 0x00, sum, FRAME_TAIL};
    HAL_UART_Transmit(&huart3, frame, 6, 100);
}

/* 云台回中：复位内部状态 + 下发复位指令给 C06B */
void gimbal_center(void) {
    gimbal.yaw = YAW_CENTER;
    gimbal.pitch = PITCH_CENTER;
    gimbal.i_yaw = 0.0f;
    gimbal.i_pitch = 0.0f;
    gimbal.last_dx = 0.0f;
    gimbal.last_dy = 0.0f;
    gimbal_reset();
}

/* ---------- 云台初始化 ---------- */
void gimbal_init(void) {
    gimbal.yaw = YAW_CENTER;
    gimbal.pitch = PITCH_CENTER;
    gimbal.i_yaw = 0.0f;
    gimbal.i_pitch = 0.0f;
    gimbal.last_dx = 0.0f;
    gimbal.last_dy = 0.0f;
    gimbal_send(gimbal.yaw, gimbal.pitch);
}

/* ---------- 云台跟踪主函数 ----------
 * 输入：K230 识别到的物体坐标 x、y（相对画面中心，右/下为正）
 * 逻辑：像素偏差 -> 比例增益 -> 目标角度 -> 限幅 -> 组帧下发 C06B
 *       位置式(不累加)：误差一减小角度自动回缩，不会像增量式那样积分过冲
 */
void gimbal_track(int16_t x, int16_t y) {
    float dx = (float)x;   /* 像素偏差，右为正 */
    float dy = (float)y;   /* 像素偏差，下为正 */

    /* 偏差变号(物体越过画面中心)：清零积分，避免旧方向的积分阻碍反向转动。
     * （这是"越偏离中心越难回归"的主因：积分记住了旧方向，反方向时拖后腿） */
    if (dx * gimbal.last_dx < 0.0f) gimbal.i_yaw   = 0.0f;
    if (dy * gimbal.last_dy < 0.0f) gimbal.i_pitch = 0.0f;
    gimbal.last_dx = dx;
    gimbal.last_dy = dy;

    /* 死区：接近中心就不再动作，防止抖动 */
    if (fabs_(dx) < DEAD_ZONE) dx = 0.0f;
    if (fabs_(dy) < DEAD_ZONE) dy = 0.0f;

    /* 积分项（抗饱和+限速）：持续累积消除稳态误差，让物体在任意角度都能居中。
     * 每帧增量限幅 INTEGRAL_STEP(=60°/s 与舵机匹配)：既快又不会在大误差时冲过头 */
    {
        float di_yaw = clamp_(KI_PIXEL * dx, -INTEGRAL_STEP, INTEGRAL_STEP);
        float di_pit = clamp_(KI_PIXEL * dy, -INTEGRAL_STEP, INTEGRAL_STEP);
        gimbal.i_yaw   += di_yaw;
        gimbal.i_pitch += di_pit;
    }
    gimbal.i_yaw   = clamp_(gimbal.i_yaw,   -INTEGRAL_MAX, INTEGRAL_MAX);
    gimbal.i_pitch = clamp_(gimbal.i_pitch, -INTEGRAL_MAX, INTEGRAL_MAX);

    if (dx == 0.0f && dy == 0.0f) return;   /* 死区内不动 */

    /* 位置式 PI + 变增益：目标角度 = 中心 + 方向 × (变增益KP×偏差 + 积分) */
    float kp_x = gain_schedule(dx);
    float kp_y = gain_schedule(dy);
    float target_yaw   = YAW_CENTER   + YAW_DIR   * (kp_x * dx + gimbal.i_yaw);
    float target_pitch = PITCH_CENTER + PITCH_DIR * (kp_y * dy + gimbal.i_pitch);

    /* 一阶低通平滑：逐步逼近目标角度，抑制抖动和过冲震荡（锁定瞬间不晃） */
    gimbal.yaw   += (target_yaw   - gimbal.yaw)   * ANGLE_SMOOTH;
    gimbal.pitch += (target_pitch - gimbal.pitch) * ANGLE_SMOOTH;

    /* 限幅后作为当前角度下发 */
    gimbal.yaw   = clamp_(gimbal.yaw,   YAW_ANGLE_MIN,   YAW_ANGLE_MAX);
    gimbal.pitch = clamp_(gimbal.pitch, PITCH_ANGLE_MIN, PITCH_ANGLE_MAX);

    printf("trk dx=%d dy=%d -> yaw=%d pit=%d\r\n",
           (int)dx, (int)dy, (int)gimbal.yaw, (int)gimbal.pitch);
    gimbal_send(gimbal.yaw, gimbal.pitch);
}

/* ---------- 距离分级（目标远近） ----------
 * A = 目标框面积占画面千分比(0~1000)，A 越小越远、A 越大越近（例 A=41 即 4.1%）。
 * 阈值需按实际目标大小整定：把目标放远/放近观察 A 值，取合适上下限 */
#define DIST_FAR_THRESH   30    /* A < 30(占画面<3%)  → 偏远 */
#define DIST_NEAR_THRESH  70    /* A > 70(占画面>7%)  → 偏近 */
#define DIST_FAR    0           /* 偏远 */
#define DIST_NORMAL 1           /* 正常距离 */
#define DIST_NEAR   2           /* 偏近 */

/* ---------- USART2 坐标解析（K230 检测帧） ----------
 * 帧格式：C:<类别>,X:<中心x>,Y:<中心y>,W:<宽>,H:<高>,S:<置信度>,A:<面积占比千分比>\r\n
 *   例：C:4,X:392,Y:374,W:80,H:200,S:92,A:41
 *   C = -1 表示目标丢失（其余字段为 0）→ 停车 + 云台回中
 */
#define GIMBAL_RX_LEN 64
static char    gimbal_rx[GIMBAL_RX_LEN];
static uint8_t gimbal_rx_cnt;
static uint8_t gimbal_lost = 0;      /* 1=当前处于目标丢失状态 */
static uint8_t dist_state = DIST_NORMAL;  /* 当前距离状态 */

/* ---------- 云台三档扫描搜索 ----------
 * 目标丢失时按三档俯仰角依次进行水平检索，循环直到锁定目标：
 *   平视档 90° → 俯视档 120° → 仰视档 80° → 平视档 ...
 * 每档先俯仰到位(短暂等待舵机到位)，再完整水平往返一次(yaw 45°↔225°)，
 * 到达水平端点后切换下一档。找到目标即停止扫描，交回跟踪。 */
#define SCAN_YAW_MIN      45.0f
#define SCAN_YAW_MAX      225.0f
#define SCAN_STEP         2.0f     /* 水平每步角度(度)：步长更小，识别更精细 */
#define SCAN_PERIOD_MS    80U     /* 每步间隔(ms)：停留更久，让K230有充足时间检测 */
#define SCAN_TIER_SETTLE  8U       /* 切换档位后等待的步数(约0.8s，让俯仰舵机到位) */

/* 三档俯仰角：平视 90° / 俯视 120° / 仰视 80°（顺序循环） */
static const float scan_pitch_tiers[3] = {90.0f, 120.0f, 80.0f};

static uint8_t scan_active = 0;   /* 1=正在扫描 */
static uint8_t scan_dir = 1;      /* 1=向225°扫, 0=向45°扫 */
static uint8_t scan_tier = 0;     /* 当前档位：0=平视 1=俯视 2=仰视 */
static uint8_t scan_settle = 0;   /* 档位切换后等待计数(>0时只等俯仰到位) */

/* ---------- 车体自转检索（独立模块，扩展水平覆盖到360°） ----------
 * 仅在目标丢失(run_flag==0)时工作：三档各扫完一次仍找不到目标，就车体原地自转，
 * 把侧方/后方转到前方再继续扫。找到目标立即退出，交回跟踪。
 * 自转角用参考轮(轮0,TIM2)编码器累计脉冲估算(无IMU)，TURN120_ENC_CNT 已标定。
 * 注意：本模块只在 run_flag==0 时运行，绝不修改跟随程序(对准/距离/PID)。 */
#define SCAN_PHASE_SEARCH 0   /* 检索阶段：三档水平往返 */
#define SCAN_PHASE_TURN   1   /* 自转阶段：车体原地左转 */
#define TURN120_ENC_CNT   1100 /* 每次自转120°参考轮累计脉冲数(实测标定: 457脉冲≈54.83°) */

static uint8_t  scan_phase   = SCAN_PHASE_SEARCH; /* 当前阶段：检索 / 自转 */
static uint8_t  turn_request = 0;                 /* 1=检索周期完成，请求车体自转 */
static uint8_t  turn_started = 0;                 /* 1=已下达左转，正在累计编码器 */
static uint32_t turn_start_cnt = 0;               /* 自转起点参考轮编码器计数 */

/* 三档扫描任务：扫描状态下每 SCAN_PERIOD_MS 走一步，非阻塞。
 * 检索阶段：每档先等俯仰舵机到位(scan_settle)，再水平 yaw 45°↔225° 完整往返一次，
 * 到达水平端点后切换到下一档(90°→120°→80°循环)；
 * 三档各扫完一次(切回平视档)即一个完整检索周期 → 请求车体自转。
 * 自转阶段：云台保持当前角度，等主循环完成车体自转。找到目标即停止。 */
void gimbal_scan_task(void) {
    static uint32_t next_scan_ms = 0;
    if (!scan_active) return;
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - next_scan_ms) < 0) return;
    next_scan_ms = now + SCAN_PERIOD_MS;

    /* 自转阶段：云台保持不动，等主循环 car_turn120_task 完成车体自转 */
    if (scan_phase == SCAN_PHASE_TURN) {
        gimbal_send(gimbal.yaw, gimbal.pitch);
        return;
    }

    /* 档位切换后的等待期：保持当前俯仰不动，让舵机到位，暂不扫水平 */
    if (scan_settle > 0) {
        scan_settle--;
        gimbal_send(gimbal.yaw, gimbal.pitch);
        return;
    }

    /* 水平扫描：yaw 45°↔225° 往返，到达端点后切换下一档并置俯仰角 */
    if (scan_dir) {
        gimbal.yaw += SCAN_STEP;
        if (gimbal.yaw >= SCAN_YAW_MAX) {
            gimbal.yaw = SCAN_YAW_MAX;
            scan_dir = 0;
            scan_tier = (scan_tier + 1) % 3;
            gimbal.pitch = scan_pitch_tiers[scan_tier];
            if (scan_tier == 0) {
                /* 三档各扫完一次 → 一个完整检索周期完成，请求车体自转 */
                scan_phase = SCAN_PHASE_TURN;
                turn_request = 1;
                turn_started = 0;
            } else {
                scan_settle = SCAN_TIER_SETTLE;
            }
        }
    } else {
        gimbal.yaw -= SCAN_STEP;
        if (gimbal.yaw <= SCAN_YAW_MIN) {
            gimbal.yaw = SCAN_YAW_MIN;
            scan_dir = 1;
            scan_tier = (scan_tier + 1) % 3;
            gimbal.pitch = scan_pitch_tiers[scan_tier];
            if (scan_tier == 0) {
                scan_phase = SCAN_PHASE_TURN;
                turn_request = 1;
                turn_started = 0;
            } else {
                scan_settle = SCAN_TIER_SETTLE;
            }
        }
    }

    gimbal_send(gimbal.yaw, gimbal.pitch);
}

/* 车体原地自转任务（非阻塞，主循环在 run_flag==0 且 turn_request==1 时调用）。
 * 每次自转 120°：用参考轮(轮0,TIM2)编码器累计脉冲估算转角，到位即停车并复位检索状态。
 * 只在目标丢失时工作；找到目标后由 gimbal_uart_feed 负责 car_stop 退出。 */
void car_turn120_task(void) {
    if (!turn_started) {
        car_turn_left();                              /* 下达原地左转指令 */
        turn_start_cnt = __HAL_TIM_GET_COUNTER(&htim2);
        turn_started = 1;
        return;
    }

    uint32_t cur   = __HAL_TIM_GET_COUNTER(&htim2);
    int32_t  delta = (int32_t)(cur - turn_start_cnt);
    uint32_t dist  = (delta < 0) ? (uint32_t)(-delta) : (uint32_t)delta;

    if (dist >= TURN120_ENC_CNT) {
        car_stop();
        turn_started = 0;
        turn_request = 0;
        printf("TURN dist=%lu (标定用)\r\n", (unsigned long)dist);
        /* 复位检索状态，从平视档起点开始新一轮三档检索 */
        scan_phase = SCAN_PHASE_SEARCH;
        scan_tier  = 0;
        scan_dir   = 1;
        gimbal.yaw    = SCAN_YAW_MIN;
        gimbal.pitch  = scan_pitch_tiers[0];
        scan_settle   = 0;
        gimbal.i_yaw = 0.0f; gimbal.i_pitch = 0.0f;
        gimbal.last_dx = 0.0f; gimbal.last_dy = 0.0f;
        gimbal_send(gimbal.yaw, gimbal.pitch);
    }
}

/* 语音指令分发函数(原型, 实现在 gimbal_uart_feed 之后)
 * K230 语音助手经 USART2 下发 V:START / V:STOP / V:ACC / V:DEC
 * 与物理按键 K0(启停)/K1(调速) 作用等效(方向语义更明确, 不循环回绕) */
static void voice_cmd_dispatch(const char *cmd);
/* F103 蜂鸣器指令(0x04帧)发送函数原型, 实现在 gimbal_uart_feed 之后 */
static void f103_buzzer_cmd(uint8_t on, uint8_t seconds);

void gimbal_uart_feed(uint8_t ch) {
    if (ch == '\r' || ch == '\n') {
        if (gimbal_rx_cnt > 0) {
            gimbal_rx[gimbal_rx_cnt] = '\0';

            /* ===== 新增: K230 语音助手下行指令, 与 C: 视觉帧共用一条线 =====
             * 帧格式: V:START<CR/LF> / V:STOP / V:ACC / V:DEC
             * START: 启动自动跟随(K0 等效); STOP: 停车并停止自转检索(K0 等效);
             * ACC: 升一档(不循环); DEC: 降一档(不循环)
             * B:<秒>: 控制 C06B(F103) 蜂鸣器(0=停, N=响N秒, 经USART3 0x04帧转发) */
            if (gimbal_rx[0] == 'B' && gimbal_rx[1] == ':')
            {
                int bsec = 0;
                if (sscanf(gimbal_rx, "B:%d", &bsec) == 1)
                    f103_buzzer_cmd((bsec > 0) ? 1U : 0U,
                                    (uint8_t)((bsec > 255) ? 255 : bsec));
                gimbal_rx_cnt = 0;
                gimbal_rx[0] = '\0';
                return;
            }
            if (gimbal_rx[0] == 'V' && gimbal_rx[1] == ':')
            {
                char vcmd[16];
                if (sscanf(gimbal_rx, "V:%15s", vcmd) == 1)
                    voice_cmd_dispatch(vcmd);
                else
                    printf("VOICE: bad cmd line\r\n");
                gimbal_rx_cnt = 0;
                gimbal_rx[0] = '\0';
                return;
            }

            int cat = -1, x = 0, y = 0, w = 0, h = 0, s = 0, a = 0;
            if (sscanf(gimbal_rx, "C:%d,X:%d,Y:%d,W:%d,H:%d,S:%d,A:%d",
                       &cat, &x, &y, &w, &h, &s, &a) == 7)
            {
                if (cat < 0)
                {
                    if (!gimbal_lost)   /* 仅在有目标→丢失的跳变时执行一次 */
                    {
                        gimbal_lost = 1;
                        run_flag = 0;          /* 停车 */
                        car_stop();
                        /* 开始三档扫描：平视90°→俯视120°→仰视80°循环，每档完整水平往返一次 */
                        scan_active = 1;
                        scan_phase = SCAN_PHASE_SEARCH;
                        turn_request = 0;
                        turn_started = 0;
                        scan_dir = 1;
                        scan_tier = 0;
                        gimbal.yaw = SCAN_YAW_MIN;
                        gimbal.pitch = scan_pitch_tiers[0];   /* 从平视档(90°)开始 */
                        scan_settle = SCAN_TIER_SETTLE;       /* 先等俯仰到位再扫水平 */
                        gimbal.i_yaw = 0.0f; gimbal.i_pitch = 0.0f;
                        gimbal.last_dx = 0.0f; gimbal.last_dy = 0.0f;
                        gimbal_send(gimbal.yaw, gimbal.pitch);
                        printf("TARGET LOST, 3-tier scan...\r\n");
                    }
                }
                else
                {
                    if (gimbal_lost) {   /* 丢失→找到的跳变：停止车体自转（若有），交回跟踪 */
                        car_stop();
                        scan_phase   = SCAN_PHASE_SEARCH;
                        turn_request = 0;
                        turn_started = 0;
                    }
                    gimbal_lost = 0;
                    scan_active = 0;   /* 找到目标，停止扫描，交给跟踪 */

                    /* 距离分级：A 越小越远、越大越近 */
                    uint8_t new_state;
                    if (a < DIST_FAR_THRESH)        new_state = DIST_FAR;
                    else if (a > DIST_NEAR_THRESH)  new_state = DIST_NEAR;
                    else                            new_state = DIST_NORMAL;
                    if (new_state != dist_state) {
                        dist_state = new_state;
                        printf("DIST A=%d -> %s\r\n", a,
                               dist_state == DIST_FAR   ? "FAR(forward)" :
                               dist_state == DIST_NEAR  ? "NEAR(backward)" : "NORMAL(stop)");
                    }

                    /* 绝对像素坐标 → 相对画面中心坐标，交给云台跟踪 */
                    gimbal_track((int16_t)(x - IMG_CENTER_X),
                                 (int16_t)(y - IMG_CENTER_Y));
                }
            }
            gimbal_rx_cnt = 0;
            gimbal_rx[0] = '\0';
        }
    } else if (gimbal_rx_cnt < GIMBAL_RX_LEN - 1) {
        gimbal_rx[gimbal_rx_cnt++] = (char)ch;
    }
}

/* ========== 语音指令分发(K230 语音助手, 等效按键 K0/K1) ==========
 *  - START: K0 启动。若正处于"目标丢失"状态则忽略(防止无目标时误跟随/乱转),
 *           需目标重新出现在画面后再喊"启动"; 若在自转检索中则继续检索。
 *  - STOP : K0 停车。立即停轮; 若正在"车体自转检索"则终止自转并回到云台三档扫描。
 *  - ACC  : K1 升一档(20/40/60/80 RPM, 到顶不再循环, 与按键循环语义略有区别)。
 *  - DEC  : K1 降一档(到底不再循环)。
 * 说明: 语音只复用 USART2 下行线, 不占用新引脚; 物理按键功能保持不变。 */
static void voice_cmd_dispatch(const char *cmd)
{
    if (cmd == NULL) return;

    if (strcmp(cmd, "START") == 0)
    {
        if (run_flag)
        {
            printf("VOICE: already running\r\n");
            return;
        }
        if (gimbal_lost)
        {
            printf("VOICE: START ignored (target lost), keep searching\r\n");
            return;
        }
        run_flag = 1;
        printf("VOICE START, speed=%d RPM\r\n", (int)speed_table[speed_level]);
    }
    else if (strcmp(cmd, "STOP") == 0)
    {
        run_flag = 0;
        car_stop();
        turn_request = 0;
        turn_started = 0;
        /* 若恰好在"车体自转检索"阶段: 停止自转, 云台回到三档扫描起点继续扫
         * (只摇头不转车, 安全) */
        if (scan_active && scan_phase == SCAN_PHASE_TURN)
        {
            scan_phase   = SCAN_PHASE_SEARCH;
            scan_tier    = 0;
            scan_dir     = 1;
            gimbal.yaw   = SCAN_YAW_MIN;
            gimbal.pitch = scan_pitch_tiers[0];
            scan_settle  = SCAN_TIER_SETTLE;
            gimbal.i_yaw = 0.0f; gimbal.i_pitch = 0.0f;
            gimbal.last_dx = 0.0f; gimbal.last_dy = 0.0f;
            gimbal_send(gimbal.yaw, gimbal.pitch);
            printf("VOICE STOP: car-turn search cancelled, gimbal scan keep\r\n");
        }
        else
        {
            printf("VOICE STOP\r\n");
        }
    }
    else if (strcmp(cmd, "ACC") == 0)
    {
        if (speed_level < LEVEL_MAX)
        {
            speed_level++;
            printf("VOICE ACC -> LEVEL %d = %d RPM\r\n",
                   (int)speed_level, (int)speed_table[speed_level]);
        }
        else
        {
            printf("VOICE ACC ignored: already max %d RPM\r\n",
                   (int)speed_table[speed_level]);
        }
    }
    else if (strcmp(cmd, "DEC") == 0)
    {
        if (speed_level > 0)
        {
            speed_level--;
            printf("VOICE DEC -> LEVEL %d = %d RPM\r\n",
                   (int)speed_level, (int)speed_table[speed_level]);
        }
        else
        {
            printf("VOICE DEC ignored: already min %d RPM\r\n",
                   (int)speed_table[speed_level]);
        }
    }
    else
    {
        printf("VOICE unknown cmd: %s\r\n", cmd);
    }
}

/* ========== F103 蜂鸣器指令(K230 B:<秒> 经 USART3 转发 C06B) ==========
 * C06B(F103) 0x04 帧格式(与其固件一致):
 *   AA 0x04 动作(1响/0停) 时长秒(0=一直响) SUM BB; SUM=(AA+04+动作+秒)&0xFF
 * 与云台帧共用 USART3, F103 按帧头/校验逐帧解析, 可交错发送。 */
static void f103_buzzer_cmd(uint8_t on, uint8_t seconds)
{
    uint8_t frame[6];
    frame[0] = 0xAA;
    frame[1] = 0x04;                 /* CMD_BUZZER */
    frame[2] = on ? 0x01 : 0x00;     /* 0=停, 1=响 */
    frame[3] = seconds;              /* 秒, 0=一直响 */
    frame[4] = (uint8_t)((frame[0] + frame[1] + frame[2] + frame[3]) & 0xFF);
    frame[5] = 0xBB;
    HAL_UART_Transmit(&huart3, frame, 6, 100);
    printf("BUZZER: %s %u s\r\n", on ? "ON" : "OFF", (unsigned)seconds);
}

/* ---------- 车身转向对齐 ----------
 * 让车头(水平135°)对准物体方位：云台跟踪到物体后，yaw 偏离中心(135°)的角度
 * 就是车头相对物体的方位偏差，用差速转向把车头转正。
 * 返回 1=已对准(可做距离跟随)，0=还在转向。
 * 方向实测反了就把 car_turn_left/right 对调。 */
#define ALIGN_DEAD_ZONE   10.0f   /* 转向死区(度)：偏差小于此值视为已对准 */

uint8_t car_align_heading(void) {
    float err = gimbal.yaw - YAW_CENTER;
    if (fabs_(err) < ALIGN_DEAD_ZONE) {
        return 1;               /* 已对准 */
    }
    if (err < 0.0f) car_turn_right();   /* yaw<135° → 物体在右 → 右转 */
    else            car_turn_left();    /* yaw>135° → 物体在左 → 左转 */
    return 0;
}

/* USART3 回环测试：短接 PB10(TX) 与 PB11(RX)，自发自收
 * 结果经 USART1(XCOM) 打印，无需 USB-TTL
 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
//uint8_t cmd;
wheel_init();
encoder_init();
wheel_pid_init();
gimbal_init();
#if WHEEL_SELFTEST_ENABLE
  wheel_selftest();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      /* 处理 USART2 收到的 K230 坐标 */
      uint8_t ch2;
      if(HAL_OK == HAL_UART_Receive(&huart2,&ch2,1,1)){
          gimbal_uart_feed(ch2);
      }

      /* 车轮闭环调速（内部按 100ms 节拍自调度，非阻塞） */
      wheel_speed_task();

      key_scan();

      gimbal_scan_task();

      if(run_flag == 1)
      {
          /* 先把车头对准物体，对准后再做距离跟随 */
          if (car_align_heading())
          {
              if (dist_state == DIST_FAR)
                  car_set_speed((float)speed_table[speed_level]);   /* 前进 */
              else if (dist_state == DIST_NEAR)
                  car_set_speed(-(float)speed_table[speed_level]);  /* 后退 */
              else
                  car_stop();   /* 正常距离 */
          }
          /* 否则 car_align_heading 里已在差速转向 */
      }
      else
      {
          /* 目标丢失时的车体自转检索（独立模块，不影响 run_flag==1 的跟随） */
          if (turn_request)
              car_turn120_task();
          else
              car_stop();
      }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
