/**
 * @file lcd_if.h
 * @brief LCD Interface Driver for 16x2 I2C LCD (HD44780 with PCF8574 I/O Expander)
 * @details Provides character LCD functions over I2C interface
 *
 * Hardware: 16x2 LCD with PCF8574 I2C backpack
 * I2C Address: 0x27 (typical) or 0x3F
 *
 * PCF8574 Pin Mapping to LCD:
 * - P0: RS (Register Select)
 * - P1: RW (Read/Write)
 * - P2: EN (Enable)
 * - P3: Backlight
 * - P4-P7: D4-D7 (4-bit data)
 */

#ifndef LCD_IF_H
#define LCD_IF_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* LCD Configuration                                                         */
/*===========================================================================*/

#define LCD_COLS                16u
#define LCD_ROWS                2u

/* Default I2C address for PCF8574 LCD backpack */
#define LCD_I2C_ADDR_DEFAULT    0x27u

/*===========================================================================*/
/* HD44780 Commands                                                          */
/*===========================================================================*/

#define LCD_CMD_CLEAR           0x01u
#define LCD_CMD_HOME            0x02u
#define LCD_CMD_ENTRY_MODE      0x04u
#define LCD_CMD_DISPLAY_CTRL    0x08u
#define LCD_CMD_CURSOR_SHIFT    0x10u
#define LCD_CMD_FUNCTION_SET    0x20u
#define LCD_CMD_SET_CGRAM       0x40u
#define LCD_CMD_SET_DDRAM       0x80u

/* Entry Mode Options */
#define LCD_ENTRY_RIGHT         0x00u
#define LCD_ENTRY_LEFT          0x02u
#define LCD_ENTRY_SHIFT_INC     0x01u
#define LCD_ENTRY_SHIFT_DEC     0x00u

/* Display Control Options */
#define LCD_DISPLAY_ON          0x04u
#define LCD_DISPLAY_OFF         0x00u
#define LCD_CURSOR_ON           0x02u
#define LCD_CURSOR_OFF          0x00u
#define LCD_BLINK_ON            0x01u
#define LCD_BLINK_OFF           0x00u

/* Function Set Options */
#define LCD_8BIT_MODE           0x10u
#define LCD_4BIT_MODE           0x00u
#define LCD_2LINE               0x08u
#define LCD_1LINE               0x00u
#define LCD_5x10_DOTS           0x04u
#define LCD_5x8_DOTS            0x00u

/*===========================================================================*/
/* PCF8574 Pin Definitions                                                   */
/*===========================================================================*/

#define LCD_PIN_RS              (1u << 0)   /* Register Select */
#define LCD_PIN_RW              (1u << 1)   /* Read/Write (0=Write) */
#define LCD_PIN_EN              (1u << 2)   /* Enable */
#define LCD_PIN_BL              (1u << 3)   /* Backlight */

/*===========================================================================*/
/* Function Prototypes                                                       */
/*===========================================================================*/

/**
 * @brief Initialize the LCD display
 * @return true on success, false on failure
 *
 * Initializes I2C and configures LCD for 4-bit mode, 2 lines, 5x8 font.
 * Turns on display with cursor off.
 */
bool LCD_Init(void);

/**
 * @brief Initialize LCD with specific I2C address
 * @param i2c_addr 7-bit I2C address of PCF8574
 * @return true on success
 */
bool LCD_InitWithAddress(uint8_t i2c_addr);

/**
 * @brief Clear the LCD display
 *
 * Clears all characters and returns cursor to home position.
 * Takes ~2ms to execute.
 */
void LCD_Clear(void);

/**
 * @brief Move cursor to home position (0,0)
 */
void LCD_Home(void);

/**
 * @brief Set cursor position
 * @param row Row number (0-1)
 * @param col Column number (0-15)
 */
void LCD_SetCursor(uint8_t row, uint8_t col);

/**
 * @brief Print a character at current cursor position
 * @param c Character to print
 */
void LCD_PrintChar(char c);

/**
 * @brief Print a null-terminated string at current cursor position
 * @param str String to print
 */
void LCD_Print(const char *str);

/**
 * @brief Print string at specific position
 * @param row Row number (0-1)
 * @param col Column number (0-15)
 * @param str String to print
 */
void LCD_PrintAt(uint8_t row, uint8_t col, const char *str);

/**
 * @brief Print integer value
 * @param value Integer to print
 */
void LCD_PrintInt(int32_t value);

/**
 * @brief Turn display on
 */
void LCD_DisplayOn(void);

/**
 * @brief Turn display off
 */
void LCD_DisplayOff(void);

/**
 * @brief Turn backlight on
 */
void LCD_BacklightOn(void);

/**
 * @brief Turn backlight off
 */
void LCD_BacklightOff(void);

/**
 * @brief Show cursor (underline)
 */
void LCD_CursorOn(void);

/**
 * @brief Hide cursor
 */
void LCD_CursorOff(void);

/**
 * @brief Enable cursor blinking
 */
void LCD_BlinkOn(void);

/**
 * @brief Disable cursor blinking
 */
void LCD_BlinkOff(void);

/**
 * @brief Create custom character
 * @param location Character code (0-7)
 * @param charmap Array of 8 bytes defining the character pattern
 */
void LCD_CreateChar(uint8_t location, const uint8_t *charmap);

/**
 * @brief Scroll display left by one position
 */
void LCD_ScrollLeft(void);

/**
 * @brief Scroll display right by one position
 */
void LCD_ScrollRight(void);

#endif /* LCD_IF_H */
