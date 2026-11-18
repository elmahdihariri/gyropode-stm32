/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   Self-balancing two-wheeled robot (gyropode) - STM32F4
  *
  *          IMU (MMA7660 + ITG-3200, I2C) -> complementary filter -> PID
  *          -> two DC motors through an L298N (TIM3 PWM).
  *          Control loop at 100 Hz, CSV telemetry on USART2 at 20 Hz.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "imu.h"
#include "pid.h"
#include "motor.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CONTROL_PERIOD_MS   10      /* 100 Hz */
#define DEBUG_PERIOD_MS     50      /* 20 Hz */

/* PID gains, tuned by hand on the robot */
#define KP                  25.0f
#define KI                  0.5f
#define KD                  2.0f
#define PID_INTEGRAL_MAX    50.0f

#define ANGLE_SETPOINT      0.0f    /* deg, 0 = position at calibration */
#define FALL_ANGLE          45.0f   /* beyond this the robot has fallen: motors off */
#define RESTART_ANGLE       5.0f    /* loop restarts once the robot is held upright again */

/* The motors don't turn below ~30 % duty (static friction), so small
   commands are raised to that minimum instead of doing nothing. */
#define MOTOR_MIN_DUTY      30

/* L298N wiring (same pins as in main.h / the .ioc) */
#define MOTOR_L_IN1_PORT    GPIOA
#define MOTOR_L_IN1_PIN     GPIO_PIN_0
#define MOTOR_L_IN2_PORT    GPIOA
#define MOTOR_L_IN2_PIN     GPIO_PIN_1
#define MOTOR_L_PWM_CH      TIM_CHANNEL_1

#define MOTOR_R_IN1_PORT    GPIOA
#define MOTOR_R_IN1_PIN     GPIO_PIN_4
#define MOTOR_R_IN2_PORT    GPIOB
#define MOTOR_R_IN2_PIN     GPIO_PIN_0
#define MOTOR_R_PWM_CH      TIM_CHANNEL_2

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static Motor motor_left = {
    .htim = &htim3, .channel = MOTOR_L_PWM_CH,
    .in1_port = MOTOR_L_IN1_PORT, .in1_pin = MOTOR_L_IN1_PIN,
    .in2_port = MOTOR_L_IN2_PORT, .in2_pin = MOTOR_L_IN2_PIN,
};

static Motor motor_right = {
    .htim = &htim3, .channel = MOTOR_R_PWM_CH,
    .in1_port = MOTOR_R_IN1_PORT, .in1_pin = MOTOR_R_IN1_PIN,
    .in2_port = MOTOR_R_IN2_PORT, .in2_pin = MOTOR_R_IN2_PIN,
};

static Pid pid_balance;
static ImuState imu;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
static void uart_print(const char *s);
static void send_telemetry(uint32_t now, float command);
static void motors_set(int16_t percent);
static int16_t compensate_friction(float command);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void uart_print(const char *s)
{
    HAL_UART_Transmit(&huart2, (const uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* one CSV line: time_ms, pitch, accel_pitch, pitch_rate, pid_output, duty */
static void send_telemetry(uint32_t now, float command)
{
    char line[96];
    snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.2f,%d\r\n",
             (unsigned long)now, imu.pitch, imu.accel_pitch, imu.pitch_rate,
             command, motor_left.command);
    uart_print(line);
}

/* Both wheels get the same command: the robot only balances, it doesn't steer. */
static void motors_set(int16_t percent)
{
    Motor_Set(&motor_left, percent);
    Motor_Set(&motor_right, percent);
}

static int16_t compensate_friction(float command)
{
    int16_t duty = (int16_t)command;

    if (duty != 0 && abs(duty) < MOTOR_MIN_DUTY)
        duty = (duty > 0) ? MOTOR_MIN_DUTY : -MOTOR_MIN_DUTY;
    return duty;
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */
    uart_print("\r\nGyropode starting\r\n");

    Motor_Init(&motor_left);
    Motor_Init(&motor_right);

    if (!Imu_Init(&hi2c1)) {
        uart_print("IMU not responding, check the I2C wiring\r\n");
        Error_Handler();
    }

    uart_print("Hold the robot upright and still: calibrating in 1 s...\r\n");
    HAL_Delay(1000);
    Imu_Calibrate();
    uart_print("Calibration done\r\n");

    Pid_Init(&pid_balance, KP, KI, KD, -100.0f, 100.0f, PID_INTEGRAL_MAX);

    uart_print("time_ms,pitch,accel_pitch,pitch_rate,pid_out,duty\r\n");

    bool running = false;
    float command = 0.0f;
    uint32_t last_control = HAL_GetTick();
    uint32_t last_debug = last_control;
    /* USER CODE END 2 */

    while (1)
    {
        /* USER CODE BEGIN 3 */
        uint32_t now = HAL_GetTick();

        if (now - last_control >= CONTROL_PERIOD_MS) {
            /* measured dt, used by both the filter and the PID */
            float dt = (float)(now - last_control) / 1000.0f;
            last_control = now;

            if (!Imu_Update(&imu, dt)) {
                motors_set(0);
                running = false;
                continue;
            }

            if (running && fabsf(imu.pitch) > FALL_ANGLE) {
                running = false;
                command = 0.0f;
                motors_set(0);
                uart_print("Fall detected, motors off\r\n");
            } else if (!running && fabsf(imu.pitch) < RESTART_ANGLE) {
                Pid_Reset(&pid_balance, imu.pitch);
                running = true;
            }

            if (running) {
                command = Pid_Update(&pid_balance, ANGLE_SETPOINT, imu.pitch, dt);
                motors_set(compensate_friction(command));
            }
        }

        if (now - last_debug >= DEBUG_PERIOD_MS) {
            last_debug = now;
            send_telemetry(now, command);
        }
        /* USER CODE END 3 */
    }
}

/**
  * @brief System Clock Configuration: HSI 16 MHz -> PLL -> 84 MHz
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 84;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief TIM3: 1 MHz counter, period 1000 -> 1 kHz PWM on CH1 and CH2
  */
static void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 84 - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief I2C1 at 100 kHz
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief USART2 at 115200 baud (ST-LINK virtual COM port on Nucleo boards)
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO clocks and the status LED (PA5). Motor pins are set up in Motor_Init().
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  Stops the motors and blinks the LED forever.
  *         Interrupts are off, so HAL_Delay() can't be used here (SysTick
  *         no longer ticks): the delay is a plain busy loop.
  */
void Error_Handler(void)
{
    __disable_irq();

    if (htim3.Instance != NULL) {   /* the timer may not be set up yet */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }

    while (1) {
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        for (volatile uint32_t i = 0; i < 1000000; i++) { }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
