#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 串口 DMA 缓冲区大小 */
#define APP_RX_BUF_SIZE                     64U

/* 电机输出参数 */
#define APP_MAX_PWM                         10000
#define APP_PWM_DEADBAND                    5
#define APP_PWM_FEEDFORWARD                 2500

/* PID 参数限制 */
#define APP_PID_INTEGRAL_LIMIT              10000.0f

/* 舵机角度 PWM */
#define APP_SERVO_CENTER                    1580
#define APP_SERVO_LOOK_RIGHT                1600

/* 状态阈值与时序 */
#define APP_STATE1_PULSE_THRESHOLD          6000LL
#define APP_STATE4_PULSE_THRESHOLD          2000LL
#define APP_STATE7_PULSE_THRESHOLD          6000LL

#define APP_STARTUP_DELAY_MS                2000U
#define APP_STOP_DELAY_MS                   1000U
#define APP_TURN_SETTLE_DELAY_MS            800U
#define APP_MAIN_LOOP_DELAY_MS              10U

/* 运动参数 */
#define APP_BASE_SPEED_FAST                 10.0f
#define APP_BASE_SPEED_SLOW                 8.0f
#define APP_TURN_ANGLE_DEG                  90.0f
#define APP_TURN_LOCK_THRESHOLD_DEG         3.0f

/* 转向 PID 限幅 */
#define APP_TURN_MAX_ROTATE                 8.0f
#define APP_TURN_MAX_DRIVE                  15.0f

/* PID 初始参数 */
#define APP_PID_LEFT_KP                     150.0f
#define APP_PID_LEFT_KI                     10.0f
#define APP_PID_LEFT_KD                     0.0f

#define APP_PID_RIGHT_KP                    150.0f
#define APP_PID_RIGHT_KI                    10.0f
#define APP_PID_RIGHT_KD                    0.0f

#define APP_PID_ANGLE_KP                    0.8f
#define APP_PID_ANGLE_KI                    0.0f
#define APP_PID_ANGLE_KD                    0.0f

#endif /* APP_CONFIG_H */
