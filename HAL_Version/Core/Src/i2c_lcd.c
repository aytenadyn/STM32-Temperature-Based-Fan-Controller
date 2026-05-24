/* ===========================================================
 * i2c_lcd.c — PCF8574 I2C LCD Sürücüsü (HD44780 uyumlu)
 * STM32 HAL tabanlı, 4-bit mod
 * =========================================================== */

#include "i2c_lcd.h"

/* ---------------------------------------------------------- */
static I2C_HandleTypeDef *_hi2c;
static uint8_t _backlight = LCD_BL;  /* Başlangıçta arka ışık açık */

/* ----------------------------------------------------------
 * Düşük seviye: PCF8574'e 1 byte yaz
 * ---------------------------------------------------------- */
static void LCD_WriteI2C(uint8_t data)
{
    HAL_I2C_Master_Transmit(_hi2c, LCD_I2C_ADDR, &data, 1, 10);
}

/* ----------------------------------------------------------
 * Enable pulse gönder (HD44780'e veriyi kilitle)
 * ---------------------------------------------------------- */
static void LCD_PulseEnable(uint8_t data)
{
    LCD_WriteI2C(data | LCD_EN);   /* EN = 1 */
    HAL_Delay(1);
    LCD_WriteI2C(data & ~LCD_EN);  /* EN = 0 */
    HAL_Delay(1);
}

/* ----------------------------------------------------------
 * 4-bit nibble gönder
 * ---------------------------------------------------------- */
static void LCD_SendNibble(uint8_t nibble, uint8_t mode)
{
    /* nibble: üst 4 bit anlamlı */
    uint8_t data = (nibble & 0xF0) | mode | _backlight;
    LCD_WriteI2C(data);
    LCD_PulseEnable(data);
}

/* ----------------------------------------------------------
 * Tam byte gönder (önce üst nibble, sonra alt nibble)
 * mode: 0 = komut, LCD_RS = veri
 * ---------------------------------------------------------- */
static void LCD_Send(uint8_t byte, uint8_t mode)
{
    LCD_SendNibble(byte & 0xF0, mode);          /* Üst 4 bit */
    LCD_SendNibble((byte << 4) & 0xF0, mode);   /* Alt 4 bit */
}

/* ============================================================
 * Public fonksiyonlar
 * ============================================================ */

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    HAL_Delay(50);  /* LCD güç-açılış bekleme */

    /* HD44780 4-bit moda geçiş prosedürü */
    LCD_SendNibble(0x30, 0); HAL_Delay(5);
    LCD_SendNibble(0x30, 0); HAL_Delay(1);
    LCD_SendNibble(0x30, 0); HAL_Delay(1);
    LCD_SendNibble(0x20, 0); HAL_Delay(1);  /* 4-bit moda geç */

    /* LCD ayarları */
    LCD_Send(LCD_FUNCTION_4BIT, 0); HAL_Delay(1);
    LCD_Send(LCD_DISPLAY_ON,    0); HAL_Delay(1);
    LCD_Send(LCD_CLEAR,         0); HAL_Delay(2);
    LCD_Send(LCD_ENTRY_MODE,    0); HAL_Delay(1);
}

void LCD_Clear(void)
{
    LCD_Send(LCD_CLEAR, 0);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t col, uint8_t row)
{
    /* Satır 0: 0x80, Satır 1: 0xC0 */
    uint8_t row_offsets[] = {0x80, 0xC0};
    LCD_Send(row_offsets[row & 0x01] + col, 0);
}

void LCD_WriteChar(char c)
{
    LCD_Send((uint8_t)c, LCD_RS);
}

void LCD_Print(const char *str)
{
    while (*str)
    {
        LCD_WriteChar(*str++);
    }
}

void LCD_Backlight(uint8_t on)
{
    _backlight = on ? LCD_BL : 0;
    LCD_WriteI2C(_backlight);
}
