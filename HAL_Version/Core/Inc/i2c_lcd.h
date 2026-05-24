/* ===========================================================
 * i2c_lcd.h — PCF8574 I2C LCD Sürücüsü (HD44780 uyumlu)
 * STM32 HAL tabanlı
 * ===========================================================
 * Kullanım:
 *   LCD_Init(&hi2c1);
 *   LCD_Print("Merhaba!");
 *   LCD_SetCursor(0, 1);
 *   LCD_Print("2. Satir");
 *   LCD_WriteChar(0xDF);  // ° sembolü
 *   LCD_Clear();
 * =========================================================== */

#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ----------------------------------------------------------
 * I2C Adresi
 * PCF8574  : genellikle 0x27 (A0=A1=A2=1)
 * PCF8574A : genellikle 0x3F (A0=A1=A2=1)
 * Modülünüzdeki jumper/solder bağlantısına göre ayarlayın.
 * HAL 8-bit adres formatı: 0x27 << 1 = 0x4E
 * ---------------------------------------------------------- */
#define LCD_I2C_ADDR   (0x27 << 1)   /* 0x4E — gerekirse 0x3F<<1=0x7E yapın */

/* PCF8574 çıkış bit tanımları (tipik modül bağlantısı) */
#define LCD_RS    (1 << 0)   /* Register Select: 0=komut, 1=veri */
#define LCD_RW    (1 << 1)   /* Read/Write: 0=yaz (her zaman 0) */
#define LCD_EN    (1 << 2)   /* Enable */
#define LCD_BL    (1 << 3)   /* Backlight */
#define LCD_D4    (1 << 4)
#define LCD_D5    (1 << 5)
#define LCD_D6    (1 << 6)
#define LCD_D7    (1 << 7)

/* LCD komutları */
#define LCD_CLEAR         0x01
#define LCD_HOME          0x02
#define LCD_ENTRY_MODE    0x06   /* artış, kaydırma yok */
#define LCD_DISPLAY_ON    0x0C   /* display açık, cursor kapalı */
#define LCD_FUNCTION_4BIT 0x28   /* 4-bit, 2 satır, 5x8 */

/* Fonksiyon prototipleri */
void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t col, uint8_t row);
void LCD_Print(const char *str);
void LCD_WriteChar(char c);
void LCD_Backlight(uint8_t on);

#endif /* I2C_LCD_H */
