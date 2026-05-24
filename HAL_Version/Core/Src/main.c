/*
 * main.c - STM32F407G-DISC1 / STM32F407VGT6
 * Project: Temperature Controlled Fan
 * HAL version
 *
 * Preserved hardware map from the previous code:
 * PA1  -> DHT11/temperature sensor DATA line, open-drain during start, input during read
 * PB10 -> TIM2_CH3 PWM, fan MOSFET gate
 * PB11 -> Passive buzzer module
 * PB6  -> I2C1_SCL, LCD SCL
 * PB9  -> I2C1_SDA, LCD SDA
 * PC1  -> RGB_R, common-anode, LOW = ON
 * PC2  -> RGB_G, common-anode, LOW = ON
 * PC4  -> RGB_B, common-anode, LOW = ON
 * USART2 -> CSV data log, only works if you have a real USART2 path to PC
 */

#include "main.h"
#include "i2c_lcd.h"
#include <stdio.h>
#include <stdint.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void delay_us(uint16_t us);
void Error_Handler(void);

/* USER CODE BEGIN PD */
#define TEMP_SENSOR_PORT        GPIOA
#define TEMP_SENSOR_PIN         GPIO_PIN_1

#define FAN_TIMER               htim2
#define FAN_CHANNEL             TIM_CHANNEL_3

#define BUZZER_PORT             GPIOB
#define BUZZER_PIN              GPIO_PIN_11

#define RGB_RED_PORT            GPIOC
#define RGB_RED_PIN             GPIO_PIN_1
#define RGB_GREEN_PORT          GPIOC
#define RGB_GREEN_PIN           GPIO_PIN_2
#define RGB_BLUE_PORT           GPIOC
#define RGB_BLUE_PIN            GPIO_PIN_4

#define TEMP_T1_X10             200U   /* 20.0 C */
#define TEMP_T2_X10             260U   /* 26.0 C */
#define TEMP_T3_X10             280U   /* 28.0 C */
#define ALARM_THRESHOLD_X10     280U   /* 28.0 C */
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static volatile uint8_t dht11_ok = 0U;
static volatile uint32_t buzzer_toggle_remaining = 0U;
static volatile uint8_t buzzer_active = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */

/* TIM4 is configured as a 1 MHz free-running counter.
 * 1 count = 1 us. This avoids direct DWT/register delay usage.
 */
static void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    while (__HAL_TIM_GET_COUNTER(&htim4) < us)
    {
        /* wait */
    }
}

static void Buzzer_Stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim3);
    buzzer_active = 0U;
    buzzer_toggle_remaining = 0U;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

static void Buzzer_Start(uint16_t freq_hz, uint16_t duration_ms)
{
    if ((freq_hz == 0U) || (duration_ms == 0U))
    {
        Buzzer_Stop();
        return;
    }

    Buzzer_Stop();

    /* TIM3 tick = 1 MHz. For square wave, interrupt frequency = 2 * tone frequency. */
    uint32_t toggle_hz = 2UL * (uint32_t)freq_hz;
    uint32_t arr = (1000000UL / toggle_hz) - 1UL;

    __HAL_TIM_SET_PRESCALER(&htim3, 83U);   /* 84 MHz / (83 + 1) = 1 MHz */
    __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    buzzer_toggle_remaining = ((uint32_t)duration_ms * toggle_hz) / 1000UL;
    if (buzzer_toggle_remaining == 0U)
    {
        buzzer_toggle_remaining = 1U;
    }

    buzzer_active = 1U;
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim3);
}

static void Buzzer_Alarm(uint8_t alarm)
{
    static uint32_t last_beep_time = 0U;
    uint32_t now = HAL_GetTick();

    if (!alarm)
    {
        Buzzer_Stop();
        return;
    }

    /* Alarm active: short 2 kHz beep every 500 ms. */
    if ((!buzzer_active) && ((now - last_beep_time) >= 500U))
    {
        last_beep_time = now;
        Buzzer_Start(2000U, 100U);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        if ((buzzer_active) && (buzzer_toggle_remaining > 0U))
        {
            HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
            buzzer_toggle_remaining--;

            if (buzzer_toggle_remaining == 0U)
            {
                Buzzer_Stop();
            }
        }
        else
        {
            Buzzer_Stop();
        }
    }
}

static void RGB_Set(uint8_t red, uint8_t green, uint8_t blue)
{
    /* Common-anode RGB LED: LOW = ON, HIGH = OFF. */
    HAL_GPIO_WritePin(RGB_RED_PORT, RGB_RED_PIN,
                      red ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(RGB_GREEN_PORT, RGB_GREEN_PIN,
                      green ? GPIO_PIN_RESET : GPIO_PIN_SET);

    HAL_GPIO_WritePin(RGB_BLUE_PORT, RGB_BLUE_PIN,
                      blue ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void RGB_Off(void)
{
    RGB_Set(0U, 0U, 0U);
}

static void Fan_SetDuty(uint8_t duty_percent)
{
    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&FAN_TIMER);
    uint32_t pulse = ((period + 1U) * duty_percent) / 100U;

    if (pulse > period)
    {
        pulse = period;
    }

    __HAL_TIM_SET_COMPARE(&FAN_TIMER, FAN_CHANNEL, pulse);
}

static void TEMP_Pin_OutputOD(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = TEMP_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TEMP_SENSOR_PORT, &GPIO_InitStruct);
}

static void TEMP_Pin_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = TEMP_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TEMP_SENSOR_PORT, &GPIO_InitStruct);
}

static uint8_t DHT11_WaitForLevel(GPIO_PinState level, uint16_t timeout_us)
{
    while (timeout_us--)
    {
        if (HAL_GPIO_ReadPin(TEMP_SENSOR_PORT, TEMP_SENSOR_PIN) == level)
        {
            return 1U;
        }
        delay_us(1U);
    }

    return 0U;
}

static uint8_t DHT11_ReadByte(void)
{
    uint8_t data = 0U;

    for (uint8_t i = 0U; i < 8U; i++)
    {
        /* Each bit starts with about 50 us LOW, then HIGH duration gives 0/1. */
        if (!DHT11_WaitForLevel(GPIO_PIN_SET, 100U))
        {
            return 0U;
        }

        delay_us(40U);
        data <<= 1U;

        if (HAL_GPIO_ReadPin(TEMP_SENSOR_PORT, TEMP_SENSOR_PIN) == GPIO_PIN_SET)
        {
            data |= 1U;
        }

        /* Wait until the sensor pulls the line LOW again. Last bit may be loose. */
        (void)DHT11_WaitForLevel(GPIO_PIN_RESET, 100U);
    }

    return data;
}

static uint16_t DHT11_ReadTemp_x10(void)
{
    static uint16_t last_valid_temp_x10 = 250U;

    uint8_t hum_int;
    uint8_t hum_dec;
    uint8_t temp_int;
    uint8_t temp_dec;
    uint8_t checksum;
    uint8_t sum;
    uint16_t temp_x10;

    /* MCU start signal: DATA low for at least 18 ms. */
    TEMP_Pin_OutputOD();
    HAL_GPIO_WritePin(TEMP_SENSOR_PORT, TEMP_SENSOR_PIN, GPIO_PIN_RESET);
    HAL_Delay(18U);

    /* Release the line and wait for DHT11 response. */
    TEMP_Pin_Input();
    delay_us(30U);

    if (!DHT11_WaitForLevel(GPIO_PIN_RESET, 100U))
    {
        dht11_ok = 0U;
        return last_valid_temp_x10;
    }

    if (!DHT11_WaitForLevel(GPIO_PIN_SET, 100U))
    {
        dht11_ok = 0U;
        return last_valid_temp_x10;
    }

    if (!DHT11_WaitForLevel(GPIO_PIN_RESET, 100U))
    {
        dht11_ok = 0U;
        return last_valid_temp_x10;
    }

    hum_int  = DHT11_ReadByte();
    hum_dec  = DHT11_ReadByte();
    temp_int = DHT11_ReadByte();
    temp_dec = DHT11_ReadByte();
    checksum = DHT11_ReadByte();

    sum = (uint8_t)(hum_int + hum_dec + temp_int + temp_dec);

    if (sum != checksum)
    {
        dht11_ok = 0U;
        return last_valid_temp_x10;
    }

    temp_x10 = (uint16_t)(temp_int * 10U);

    if (temp_dec <= 9U)
    {
        temp_x10 += temp_dec;
    }

    if (temp_x10 > 800U)
    {
        dht11_ok = 0U;
        return last_valid_temp_x10;
    }

    dht11_ok = 1U;
    last_valid_temp_x10 = temp_x10;

    return last_valid_temp_x10;
}

static uint16_t Temperature_Filter(uint16_t new_temp_x10)
{
    static uint8_t first_read = 1U;
    static uint16_t filtered_temp_x10 = 0U;

    if (first_read)
    {
        filtered_temp_x10 = new_temp_x10;
        first_read = 0U;
    }
    else
    {
        filtered_temp_x10 = (uint16_t)((filtered_temp_x10 + new_temp_x10) / 2U);
    }

    return filtered_temp_x10;
}

static uint8_t Alarm_Update(uint16_t temp_x10)
{
    return (temp_x10 >= ALARM_THRESHOLD_X10) ? 1U : 0U;
}

static uint8_t Get_Zone(uint16_t temp_x10)
{
    if (temp_x10 < TEMP_T1_X10)
    {
        return 1U;
    }
    else if (temp_x10 < TEMP_T2_X10)
    {
        return 2U;
    }
    else if (temp_x10 < TEMP_T3_X10)
    {
        return 3U;
    }
    else
    {
        return 4U;
    }
}

static uint8_t Control_Fan_And_RGB(uint16_t temp_x10)
{
    uint8_t duty;

    if (temp_x10 < TEMP_T1_X10)
    {
        duty = 0U;
        RGB_Set(0U, 0U, 1U);     /* Blue */
    }
    else if (temp_x10 < TEMP_T2_X10)
    {
        duty = 35U;
        RGB_Set(0U, 1U, 0U);     /* Green */
    }
    else if (temp_x10 < TEMP_T3_X10)
    {
        duty = 70U;
        RGB_Set(1U, 1U, 0U);     /* Yellow */
    }
    else
    {
        duty = 100U;
        RGB_Set(1U, 0U, 0U);     /* Red */
    }

    Fan_SetDuty(duty);
    return duty;
}

static void LCD_Update(uint16_t temp_x10, uint8_t duty, uint8_t alarm)
{
    char line1[17];
    char line2[17];

    snprintf(line1, sizeof(line1), "Temp:%2u.%1u C     ",
             temp_x10 / 10U,
             temp_x10 % 10U);

    if (dht11_ok)
    {
        snprintf(line2, sizeof(line2), "Fan:%3u%% AL:%u    ", duty, alarm);
    }
    else
    {
        snprintf(line2, sizeof(line2), "DHT11 ERR       ");
    }

    LCD_SetCursor(0U, 0U);
    LCD_Print(line1);
    LCD_SetCursor(0U, 1U);
    LCD_Print(line2);
}

static void UART_LogData(uint16_t temp_x10,
                         uint8_t duty,
                         uint8_t zone,
                         uint8_t alarm)
{
    char msg[96];

    int len = snprintf(msg, sizeof(msg),
                       "%lu,%u.%u,%u,%u,%u\r\n",
                       HAL_GetTick(),
                       temp_x10 / 10U,
                       temp_x10 % 10U,
                       duty,
                       zone,
                       alarm);

    HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)len, 100U);
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();

    HAL_TIM_Base_Start(&htim4);                 /* 1 us delay timer */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);   /* fan PWM */

    Buzzer_Stop();
    Fan_SetDuty(0U);
    RGB_Off();

    LCD_Init(&hi2c1);
    LCD_Clear();
    LCD_SetCursor(0U, 0U);
    LCD_Print("System Ready");
    LCD_SetCursor(0U, 1U);
    LCD_Print("Starting...");
    HAL_Delay(700U);

    char header[] = "time_ms,temp_c,fan_duty_%,zone,alarm\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)header, sizeof(header) - 1U, 100U);

    while (1)
    {
        uint16_t raw_temp_x10 = DHT11_ReadTemp_x10();
        uint16_t filtered_temp_x10 = Temperature_Filter(raw_temp_x10);
        uint8_t fan_duty = Control_Fan_And_RGB(filtered_temp_x10);
        uint8_t zone = Get_Zone(filtered_temp_x10);
        uint8_t alarm = Alarm_Update(raw_temp_x10);

        Buzzer_Alarm(alarm);
        LCD_Update(filtered_temp_x10, fan_duty, alarm);
        UART_LogData(filtered_temp_x10, fan_duty, zone, alarm);

        HAL_Delay(1200U);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

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

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4199;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 499;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

static void MX_TIM4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 83;                  /* 84 MHz / 84 = 1 MHz */
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
}

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

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* Initial levels */
    HAL_GPIO_WritePin(TEMP_SENSOR_PORT, TEMP_SENSOR_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_RED_PORT, RGB_RED_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_GREEN_PORT, RGB_GREEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RGB_BLUE_PORT, RGB_BLUE_PIN, GPIO_PIN_SET);

    /* PA1 temperature sensor data line. It is reconfigured as input during read. */
    GPIO_InitStruct.Pin = TEMP_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TEMP_SENSOR_PORT, &GPIO_InitStruct);

    /* PB11 buzzer output */
    GPIO_InitStruct.Pin = BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

    /* PC1, PC2, PC4 RGB outputs */
    GPIO_InitStruct.Pin = RGB_RED_PIN | RGB_GREEN_PIN | RGB_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
