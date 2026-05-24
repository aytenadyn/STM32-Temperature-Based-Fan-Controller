/*
 * main.c - STM32F407G-DISC1 / STM32F407VGT6
 * Project: Temperature-Based DC Fan Controller
 *
 * Description:
 * This project implements a temperature-controlled cooling system using
 * bare-metal C programming on the STM32F407G-DISC1 board. The system reads
 * temperature data from a DHT11 sensor, controls a DC fan using PWM, displays
 * system status on an I2C LCD, indicates temperature zones with an RGB LED,
 * and activates a buzzer alarm when the temperature exceeds the threshold.
 *
 * Programming approach:
 * - Bare-metal C
 * - Direct register access
 * - No HAL functions are used in the application logic
 * - STM32CubeIDE/.ioc is used only for project setup, startup files,
 *   linker script, and pin reference
 *
 * Clock configuration:
 * - SYSCLK = 84 MHz
 * - PLL source = HSI
 * - PLLM = 16, PLLN = 336, PLLP = /4, PLLQ = 7
 * - APB1 = 42 MHz
 * - APB2 = 84 MHz
 * - TIM2 timer clock = 84 MHz because APB1 prescaler is /2
 *
 * Pin map:
 * - PA1  -> DHT11 DATA, GPIO open-drain output / input during read
 * - PB10 -> FAN_PWM, TIM2_CH3 PWM output, MOSFET gate control
 * - PB11 -> BUZZER, GPIO push-pull output
 * - PB6  -> LCD SCL, I2C1_SCL AF4 open-drain
 * - PB9  -> LCD SDA, I2C1_SDA AF4 open-drain
 * - PC1  -> RGB Red, common-anode LED, LOW = ON
 * - PC2  -> RGB Green, common-anode LED, LOW = ON
 * - PC4  -> RGB Blue, common-anode LED, LOW = ON
 *
 */

#include "stm32f407xx.h"
#include <stdint.h>
#include <stdio.h>

/* ============================================================
 *                  PROJECT CONSTANTS AND PIN DEFINITIONS
 * ============================================================ */

#define SYSCLK_HZ               84000000UL
#define APB1_HZ                 42000000UL
#define TIM2_CLK_HZ             84000000UL

#define TEMP_SENSOR_PORT GPIOA
#define TEMP_SENSOR_PIN  1U

#define FAN_TIMER               TIM2
#define FAN_PWM_ARR             4199U

#define BUZZER_PORT             GPIOB
#define BUZZER_PIN              11U

#define RGB_PORT                GPIOC
#define RGB_RED_PIN             1U
#define RGB_GREEN_PIN           2U
#define RGB_BLUE_PIN            4U

#define LCD_I2C_ADDR            0x27U
#define LCD_BACKLIGHT           0x08U
#define LCD_EN                  0x04U
#define LCD_RS                  0x01U

#define TEMP_T1_X10             200U   /* 20.0°C */
#define TEMP_T2_X10             260U   /* 26.0°C */
#define TEMP_T3_X10             280U   /* 28.0°C */
#define ALARM_THRESHOLD_X10     280U   /* 28.0°C */

/* ============================================================
 *                  FUNCTION PROTOTYPES AND GLOBAL VARIABLES
 * ============================================================ */

static uint8_t gpio_read(GPIO_TypeDef *port, uint32_t pin);
static void Buzzer_Stop(void);

static volatile uint8_t temp_sensor_ok = 0U;

/* ============================================================
 *                         ITM / SWO LOGGING
 * ============================================================ */

static void ITM_SendChar_SWO(char c)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&
        (ITM->TER & 1UL))
    {
        while (ITM->PORT[0].u32 == 0UL) { }
        ITM->PORT[0].u8 = (uint8_t)c;
    }
}

static void ITM_Log(const char *s)
{
    while (*s)
    {
        ITM_SendChar_SWO(*s++);
    }
}

static void ITM_LogData(uint32_t timestamp_ms,
                        uint16_t temp_x10,
                        uint8_t duty,
                        uint8_t zone,
                        uint8_t alarm)
{
    char buf[80];

    /*
     * CSV format:
     * time_ms,temp_c,fan_duty_%,zone,alarm
     *
     * zone:
     * 1 -> low temperature
     * 2 -> medium temperature
     * 3 -> high temperature
     * 4 -> very high temperature
     */
    snprintf(buf, sizeof(buf),
             "%lu,%u.%u,%u,%u,%u\r\n",
             timestamp_ms,
             temp_x10 / 10U,
             temp_x10 % 10U,
             duty,
             zone,
             alarm);

    ITM_Log(buf);
}

/* ============================================================
 *                         DELAY AND TIMING
 * ============================================================ */

static uint32_t get_uptime_ms(void)
{
    return (uint32_t)(DWT->CYCCNT / (SYSCLK_HZ / 1000UL));
}

static void DWT_Delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SYSCLK_HZ / 1000000UL);

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms--)
    {
        delay_us(1000U);
    }
}

static uint32_t dwt_cycles(void)
{
    return DWT->CYCCNT;
}

/* ============================================================
 *                      SYSTEM CLOCK CONFIGURATION
 * ============================================================ */

static void Clock_Init_84MHz(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { }

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    FLASH->ACR = FLASH_ACR_ICEN |
                 FLASH_ACR_DCEN |
                 FLASH_ACR_PRFTEN |
                 FLASH_ACR_LATENCY_2WS;

    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY) { }

    RCC->PLLCFGR = (16U << 0U) |
                   (336U << 6U) |
                   (1U << 16U) |
                   (7U << 24U);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { }

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { }

    SystemCoreClock = SYSCLK_HZ;
}

/* ============================================================
 *                         GPIO UTILITIES
 * ============================================================ */

static void gpio_set(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = (1UL << pin);
}

static void gpio_reset(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = (1UL << (pin + 16U));
}

static uint8_t gpio_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR & (1UL << pin)) ? 1U : 0U;
}

static void gpio_output_pp(GPIO_TypeDef *port, uint32_t pin)
{
    port->MODER &= ~(3UL << (pin * 2U));
    port->MODER |= (1UL << (pin * 2U));
    port->OTYPER &= ~(1UL << pin);
    port->OSPEEDR &= ~(3UL << (pin * 2U));
    port->PUPDR &= ~(3UL << (pin * 2U));
}

/* ============================================================
 *                    PROJECT GPIO INITIALIZATION
 * ============================================================ */

static void GPIO_Project_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;

    gpio_output_pp(BUZZER_PORT, BUZZER_PIN);
    gpio_reset(BUZZER_PORT, BUZZER_PIN);

    gpio_output_pp(RGB_PORT, RGB_RED_PIN);
    gpio_output_pp(RGB_PORT, RGB_GREEN_PIN);
    gpio_output_pp(RGB_PORT, RGB_BLUE_PIN);

    gpio_set(RGB_PORT, RGB_RED_PIN);
    gpio_set(RGB_PORT, RGB_GREEN_PIN);
    gpio_set(RGB_PORT, RGB_BLUE_PIN);
}

/* ============================================================
 *                         FAN PWM CONTROL
 * ============================================================ */

static void Fan_PWM_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    GPIOB->MODER &= ~(3UL << (10U * 2U));
    GPIOB->MODER |= (2UL << (10U * 2U));

    GPIOB->AFR[1] &= ~(0xFUL << ((10U - 8U) * 4U));
    GPIOB->AFR[1] |= (1UL << ((10U - 8U) * 4U));

    GPIOB->OTYPER &= ~(1UL << 10U);
    GPIOB->OSPEEDR |= (3UL << (10U * 2U));
    GPIOB->PUPDR &= ~(3UL << (10U * 2U));

    TIM2->PSC = 0U;
    TIM2->ARR = FAN_PWM_ARR;
    TIM2->CCR3 = 0U;

    TIM2->CCMR2 &= ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M);
    TIM2->CCMR2 |= (TIM_CCMR2_OC3M_1 |
                    TIM_CCMR2_OC3M_2 |
                    TIM_CCMR2_OC3PE);

    TIM2->CCER |= TIM_CCER_CC3E;
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/*
 * Sets PWM duty cycle for the fan.
 * TIM2_CH3 is connected to the MOSFET gate.
 * CCR3 determines how long the PWM signal stays HIGH in one period.
 */
static void Fan_SetDuty(uint8_t duty_percent)
{
    if (duty_percent > 100U)
    {
        duty_percent = 100U;
    }

    FAN_TIMER->CCR3 = ((FAN_PWM_ARR + 1U) * duty_percent) / 100U;
}

/* ============================================================
 *                         I2C LCD DRIVER
 * ============================================================ */

static void I2C1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~((3UL << (6U * 2U)) | (3UL << (9U * 2U)));
    GPIOB->MODER |= ((2UL << (6U * 2U)) | (2UL << (9U * 2U)));

    GPIOB->OTYPER |= (1UL << 6U) | (1UL << 9U);
    GPIOB->OSPEEDR |= (3UL << (6U * 2U)) | (3UL << (9U * 2U));

    GPIOB->PUPDR &= ~((3UL << (6U * 2U)) | (3UL << (9U * 2U)));
    GPIOB->PUPDR |= ((1UL << (6U * 2U)) | (1UL << (9U * 2U)));

    GPIOB->AFR[0] &= ~(0xFUL << (6U * 4U));
    GPIOB->AFR[0] |= (4UL << (6U * 4U));

    GPIOB->AFR[1] &= ~(0xFUL << ((9U - 8U) * 4U));
    GPIOB->AFR[1] |= (4UL << ((9U - 8U) * 4U));

    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 42U;
    I2C1->CCR = 210U;
    I2C1->TRISE = 43U;
    I2C1->CR1 |= I2C_CR1_PE;
}

static uint8_t I2C1_WriteByte(uint8_t addr7, uint8_t data)
{
    uint32_t timeout;

    timeout = 100000U;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && --timeout) { }
    if (timeout == 0U) return 0U;

    I2C1->CR1 |= I2C_CR1_START;

    timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_SB) && --timeout) { }
    if (timeout == 0U) return 0U;

    I2C1->DR = (uint8_t)(addr7 << 1U);

    timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
    {
        if (I2C1->SR1 & I2C_SR1_AF)
        {
            I2C1->SR1 &= ~I2C_SR1_AF;
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }

        if (--timeout == 0U)
        {
            I2C1->CR1 |= I2C_CR1_STOP;
            return 0U;
        }
    }

    (void)I2C1->SR1;
    (void)I2C1->SR2;

    timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_TXE) && --timeout) { }
    if (timeout == 0U)
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        return 0U;
    }

    I2C1->DR = data;

    timeout = 100000U;
    while (!(I2C1->SR1 & I2C_SR1_BTF) && --timeout) { }
    if (timeout == 0U)
    {
        I2C1->CR1 |= I2C_CR1_STOP;
        return 0U;
    }

    I2C1->CR1 |= I2C_CR1_STOP;
    return 1U;
}

static void LCD_Write4(uint8_t nibble, uint8_t rs)
{
    uint8_t data = (uint8_t)((nibble << 4U) |
                             LCD_BACKLIGHT |
                             (rs ? LCD_RS : 0U));

    I2C1_WriteByte(LCD_I2C_ADDR, data | LCD_EN);
    delay_us(2U);
    I2C1_WriteByte(LCD_I2C_ADDR, data & (uint8_t)~LCD_EN);
    delay_us(50U);
}

/*
 * Sends one byte to LCD.
 * rs = 0 -> command
 * rs = 1 -> character data
 * LCD works in 4-bit mode, so the byte is sent as two nibbles.
 */
static void LCD_Send(uint8_t value, uint8_t rs)
{
    LCD_Write4((uint8_t)(value >> 4U), rs);
    LCD_Write4((uint8_t)(value & 0x0FU), rs);
}

static void LCD_Command(uint8_t cmd)
{
    LCD_Send(cmd, 0U);

    if ((cmd == 0x01U) || (cmd == 0x02U))
    {
        delay_ms(2U);
    }
}

static void LCD_Data(uint8_t data)
{
    LCD_Send(data, 1U);
}

static void LCD_Init(void)
{
    delay_ms(50U);

    LCD_Write4(0x03U, 0U);
    delay_ms(5U);

    LCD_Write4(0x03U, 0U);
    delay_us(150U);

    LCD_Write4(0x03U, 0U);
    LCD_Write4(0x02U, 0U);

    LCD_Command(0x28U);
    LCD_Command(0x0CU);
    LCD_Command(0x06U);
    LCD_Command(0x01U);
}

static void LCD_Clear(void)
{
    LCD_Command(0x01U);
}

static void LCD_SetCursor(uint8_t col, uint8_t row)
{
    static const uint8_t row_offsets[2] = {0x00U, 0x40U};

    if (row > 1U)
    {
        row = 1U;
    }

    LCD_Command((uint8_t)(0x80U | (col + row_offsets[row])));
}

static void LCD_Print(const char *s)
{
    while (*s)
    {
        LCD_Data((uint8_t)*s++);
    }
}

/*
 * Updates the 16x2 LCD.
 * First row shows filtered temperature.
 * Second row shows fan duty and alarm state.
 */
static void LCD_Update(uint16_t temp_x10, uint8_t duty, uint8_t alarm)
{
    char line1[17];
    char line2[17];

    snprintf(line1, sizeof(line1), "Temp:%2u.%1u C     ",
             temp_x10 / 10U,
             temp_x10 % 10U);

    if (temp_sensor_ok)
    {
        snprintf(line2, sizeof(line2), "Fan:%3u%% AL:%u    ",
                 duty,
                 alarm);
    }
    else
    {
        snprintf(line2, sizeof(line2), "DHT11 ERR     ");
    }

    LCD_SetCursor(0U, 0U);
    LCD_Print(line1);

    LCD_SetCursor(0U, 1U);
    LCD_Print(line2);
}

/* ============================================================
 *                         RGB LED CONTROL
 * ============================================================ */

static void RGB_Set(uint8_t red, uint8_t green, uint8_t blue)
{
    red   ? gpio_reset(RGB_PORT, RGB_RED_PIN)   : gpio_set(RGB_PORT, RGB_RED_PIN);
    green ? gpio_reset(RGB_PORT, RGB_GREEN_PIN) : gpio_set(RGB_PORT, RGB_GREEN_PIN);
    blue  ? gpio_reset(RGB_PORT, RGB_BLUE_PIN)  : gpio_set(RGB_PORT, RGB_BLUE_PIN);
}

static void RGB_Off(void)
{
    RGB_Set(0U, 0U, 0U);
}
/* ============================================================
 *                         STARTUP DISPLAY
 * ============================================================ */


static void LCD_Show_Ready(void)
{
    Fan_SetDuty(0U);
    RGB_Off();
    Buzzer_Stop();

    LCD_Clear();
    LCD_SetCursor(0U, 0U);
    LCD_Print("System Ready");

    LCD_SetCursor(0U, 1U);
    LCD_Print("Starting...");
}

/* ============================================================
 *                         BUZZER CONTROL
 * ============================================================ */

static volatile uint32_t buzzer_toggle_remaining = 0U;
static volatile uint8_t buzzer_pin_state = 0U;
static volatile uint8_t buzzer_active = 0U;

static void Buzzer_Timer_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->CR1 = 0U;
    TIM3->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM3_IRQn, 3U);
    NVIC_EnableIRQ(TIM3_IRQn);
}

static void Buzzer_Stop(void)
{
    TIM3->CR1 &= ~TIM_CR1_CEN;

    buzzer_active = 0U;
    buzzer_toggle_remaining = 0U;
    buzzer_pin_state = 0U;

    gpio_reset(BUZZER_PORT, BUZZER_PIN);
}

static void Buzzer_Start(uint16_t freq_hz, uint16_t duration_ms)
{
    if ((freq_hz == 0U) || (duration_ms == 0U))
    {
        Buzzer_Stop();
        return;
    }

    Buzzer_Stop();

    uint32_t toggle_hz = 2UL * freq_hz;
    uint32_t arr = (1000000UL / toggle_hz) - 1UL;

    TIM3->PSC = 83U;
    TIM3->ARR = arr;
    TIM3->CNT = 0U;

    buzzer_toggle_remaining = ((uint32_t)duration_ms * toggle_hz) / 1000UL;

    if (buzzer_toggle_remaining == 0U)
    {
        buzzer_toggle_remaining = 1U;
    }

    buzzer_active = 1U;
    buzzer_pin_state = 0U;

    TIM3->SR = 0U;
    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR = 0U;
    TIM3->CR1 |= TIM_CR1_CEN;
}

void TIM3_IRQHandler(void)
{
    if (TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;

        if ((buzzer_active) && (buzzer_toggle_remaining > 0U))
        {
            buzzer_pin_state ^= 1U;

            if (buzzer_pin_state)
            {
                gpio_set(BUZZER_PORT, BUZZER_PIN);
            }
            else
            {
                gpio_reset(BUZZER_PORT, BUZZER_PIN);
            }

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

static void Buzzer_Alarm(uint8_t alarm)
{
    static uint32_t last_beep_cycle = 0U;
    uint32_t now = dwt_cycles();

    if (!alarm)
    {
        Buzzer_Stop();
        return;
    }

    if ((!buzzer_active) && ((now - last_beep_cycle) >= (SYSCLK_HZ / 2U)))
    {
        last_beep_cycle = now;
        Buzzer_Start(2000U, 100U);
    }
}

/* ============================================================
 *                         DHT11 SENSOR DRIVER
 * ============================================================ */

static void DHT11_Pin_OutputOD(void)
{
	TEMP_SENSOR_PORT->MODER &= ~(3UL << (TEMP_SENSOR_PIN * 2U));
	TEMP_SENSOR_PORT->MODER |=  (1UL << (TEMP_SENSOR_PIN * 2U));

	TEMP_SENSOR_PORT->OTYPER |= (1UL << TEMP_SENSOR_PIN);

	TEMP_SENSOR_PORT->OSPEEDR &= ~(3UL << (TEMP_SENSOR_PIN * 2U));
	TEMP_SENSOR_PORT->PUPDR &= ~(3UL << (TEMP_SENSOR_PIN * 2U));
}

static void DHT11_Pin_Input(void)
{
	TEMP_SENSOR_PORT->MODER &= ~(3UL << (TEMP_SENSOR_PIN * 2U));
	TEMP_SENSOR_PORT->PUPDR &= ~(3UL << (TEMP_SENSOR_PIN * 2U));
}

static uint8_t DHT11_WaitForLevel(uint8_t level, uint32_t timeout_us)
{
    while (timeout_us--)
    {
        if (gpio_read(TEMP_SENSOR_PORT, TEMP_SENSOR_PIN) == level)
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


        if (!DHT11_WaitForLevel(1U, 100U))
        {
            return 0U;
        }

        delay_us(40U);

        data <<= 1U;

        if (gpio_read(TEMP_SENSOR_PORT, TEMP_SENSOR_PIN))
        {
            data |= 1U;
        }

        if (!DHT11_WaitForLevel(0U, 100U))
        {
        }
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

    /*
     * DHT11 start signal:
     */
    DHT11_Pin_OutputOD();
    gpio_reset(TEMP_SENSOR_PORT, TEMP_SENSOR_PIN);
    delay_ms(18U);


    DHT11_Pin_Input();
    delay_us(30U);

    /*
     * DHT11 response:
     * LOW ~80 us, HIGH ~80 us
     */
    if (!DHT11_WaitForLevel(0U, 100U))
    {
    	temp_sensor_ok = 0U;
        return last_valid_temp_x10;
    }

    if (!DHT11_WaitForLevel(1U, 100U))
    {
    	temp_sensor_ok = 0U;
        return last_valid_temp_x10;
    }

    if (!DHT11_WaitForLevel(0U, 100U))
    {
    	temp_sensor_ok = 0U;
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
    	temp_sensor_ok = 0U;
        return last_valid_temp_x10;
    }

    temp_x10 = (uint16_t)(temp_int * 10U);

    if (temp_dec <= 9U)
    {
        temp_x10 += temp_dec;
    }

    if ((temp_x10 < 0U) || (temp_x10 > 800U))
    {
    	temp_sensor_ok = 0U;
        return last_valid_temp_x10;
    }

    temp_sensor_ok = 1U;
    last_valid_temp_x10 = temp_x10;

    return last_valid_temp_x10;
}

/* ============================================================
 *                         CONTROL LOGIC
 * ============================================================ */

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
    if (temp_x10 >= ALARM_THRESHOLD_X10)
    {
        return 1U;
    }
    else
    {
        return 0U;
    }
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

/*
 * Temperature-based fan and RGB control.
 *
 * temp_x10 format is used to avoid floating point operations.
 * Example: 253 means 25.3°C.
 *
 * Temperature zones:
 *   T < 20°C       -> Fan OFF, Blue LED
 *   20°C - 25.9°C -> 35% duty, Green LED
 *   26°C - 27.9°C -> 70% duty, Yellow LED
 *   T >= 28°C     -> 100% duty, Red LED
 */
static uint8_t Control_Fan_And_RGB(uint16_t temp_x10)
{
    uint8_t duty;

    if (temp_x10 < TEMP_T1_X10)
    {
        duty = 0U;
        RGB_Set(0U, 0U, 1U);
    }
    else if (temp_x10 < TEMP_T2_X10)
    {
        duty = 35U;
        RGB_Set(0U, 1U, 0U);
    }
    else if (temp_x10 < TEMP_T3_X10)
    {
        duty = 70U;
        RGB_Set(1U, 1U, 0U);
    }
    else
    {
        duty = 100U;
        RGB_Set(1U, 0U, 0U);
    }

    Fan_SetDuty(duty);

    return duty;
}

/* ============================================================
 *                         MAIN APPLICATION
 * ============================================================ */

int main(void)
{
    Clock_Init_84MHz();
    DWT_Delay_Init();

    GPIO_Project_Init();
    Buzzer_Timer_Init();
    Fan_PWM_Init();
    I2C1_Init();

    LCD_Init();
    LCD_Show_Ready();

    delay_ms(700U);

    ITM_Log("time_ms,temp_c,fan_duty_%,zone,alarm\r\n");

    while (1)
    {
    	uint16_t raw_temp_x10 = DHT11_ReadTemp_x10();
    	uint16_t filtered_temp_x10 = Temperature_Filter(raw_temp_x10);

    	uint8_t fan_duty = Control_Fan_And_RGB(filtered_temp_x10);
    	uint8_t zone = Get_Zone(filtered_temp_x10);
    	uint8_t alarm = Alarm_Update(filtered_temp_x10);

    	Buzzer_Alarm(alarm);
    	LCD_Update(filtered_temp_x10, fan_duty, alarm);
    	ITM_LogData(get_uptime_ms(), filtered_temp_x10, fan_duty, zone, alarm);

        delay_ms(1200U);
    }
}
