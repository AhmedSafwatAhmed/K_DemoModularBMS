/**
 * @file lcd_if.c
 * @brief LCD Interface Driver Implementation
 * @details I2C-based 16x2 LCD driver using PCF8574 I/O expander
 */

#include "lcd_if.h"
#include "i2c.h"
#include "timer.h"
#include <stdio.h>

/*===========================================================================*/
/* Private Variables                                                         */
/*===========================================================================*/

static uint8_t lcd_i2c_addr = LCD_I2C_ADDR_DEFAULT;
static uint8_t lcd_backlight = LCD_PIN_BL;
static uint8_t lcd_display_ctrl = LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
static bool lcd_initialized = false;

/*===========================================================================*/
/* Private Functions                                                         */
/*===========================================================================*/

/**
 * @brief Write byte to PCF8574 (LCD interface)
 */
static void lcd_write_i2c(uint8_t data) {
    I2C_WriteByte(lcd_i2c_addr, data | lcd_backlight);
}

/**
 * @brief Pulse the Enable pin to latch data
 */
static void lcd_pulse_enable(uint8_t data) {
    lcd_write_i2c(data | LCD_PIN_EN);  /* EN high */
    TIMER_DelayUs(1);
    lcd_write_i2c(data & ~LCD_PIN_EN);  /* EN low */
    TIMER_DelayUs(50);  /* Commands need >37us to settle */
}

/**
 * @brief Send 4 bits to LCD
 */
static void lcd_write_4bits(uint8_t value) {
    lcd_write_i2c(value);
    lcd_pulse_enable(value);
}

/**
 * @brief Send byte to LCD (as two 4-bit nibbles)
 * @param value Byte to send
 * @param rs_mode 0 for command, LCD_PIN_RS for data
 */
static void lcd_send_byte(uint8_t value, uint8_t rs_mode) {
    uint8_t high_nibble = (value & 0xF0) | rs_mode;
    uint8_t low_nibble = ((value << 4) & 0xF0) | rs_mode;

    lcd_write_4bits(high_nibble);
    lcd_write_4bits(low_nibble);
}

/**
 * @brief Send command to LCD
 */
static void lcd_command(uint8_t cmd) {
    lcd_send_byte(cmd, 0);
}

/**
 * @brief Send data (character) to LCD
 */
static void lcd_data(uint8_t data) {
    lcd_send_byte(data, LCD_PIN_RS);
}

/*===========================================================================*/
/* Public Function Implementations                                           */
/*===========================================================================*/

bool LCD_Init(void) {
    return LCD_InitWithAddress(LCD_I2C_ADDR_DEFAULT);
}

bool LCD_InitWithAddress(uint8_t i2c_addr) {
    lcd_i2c_addr = i2c_addr;

    /* Initialize I2C */
    I2C_Config_t i2c_config = {
        .clock_hz = 100000u  /* 100kHz standard mode */
    };

    if (I2C_Init(&i2c_config) != I2C_OK) {
        return false;
    }

    /* Wait for LCD power-up (>40ms after Vcc rises to 2.7V) */
    TIMER_DelayMs(50);

    /* Initialize LCD in 4-bit mode
     * Following HD44780 initialization sequence
     */

    /* Set backlight on initially */
    lcd_backlight = LCD_PIN_BL;
    lcd_write_i2c(0x00);

    TIMER_DelayMs(5);

    /* Send 0x30 three times to ensure 8-bit mode, then switch to 4-bit */

    /* First try: Function set (8-bit mode) */
    lcd_write_4bits(0x30);
    TIMER_DelayMs(5);  /* Wait >4.1ms */

    /* Second try */
    lcd_write_4bits(0x30);
    TIMER_DelayUs(150);  /* Wait >100us */

    /* Third try */
    lcd_write_4bits(0x30);
    TIMER_DelayUs(150);

    /* Now switch to 4-bit mode */
    lcd_write_4bits(0x20);
    TIMER_DelayUs(150);

    /* Now in 4-bit mode, can use lcd_command() */

    /* Function set: 4-bit mode, 2 lines, 5x8 font */
    lcd_command(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);

    /* Display control: display on, cursor off, blink off */
    lcd_display_ctrl = LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF;
    lcd_command(LCD_CMD_DISPLAY_CTRL | lcd_display_ctrl);

    /* Clear display */
    lcd_command(LCD_CMD_CLEAR);
    TIMER_DelayMs(2);  /* Clear takes ~1.52ms */

    /* Entry mode: increment cursor, no shift */
    lcd_command(LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT);

    lcd_initialized = true;

    return true;
}

void LCD_Clear(void) {
    if (!lcd_initialized) return;
    lcd_command(LCD_CMD_CLEAR);
    TIMER_DelayMs(2);
}

void LCD_Home(void) {
    if (!lcd_initialized) return;
    lcd_command(LCD_CMD_HOME);
    TIMER_DelayMs(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    if (!lcd_initialized) return;

    /* Row offsets for 16x2 LCD */
    static const uint8_t row_offsets[] = {0x00, 0x40};

    if (row >= LCD_ROWS) {
        row = LCD_ROWS - 1u;
    }
    if (col >= LCD_COLS) {
        col = LCD_COLS - 1u;
    }

    lcd_command(LCD_CMD_SET_DDRAM | (col + row_offsets[row]));
}

void LCD_PrintChar(char c) {
    if (!lcd_initialized) return;
    lcd_data((uint8_t)c);
}

void LCD_Print(const char *str) {
    if (!lcd_initialized || str == NULL) return;

    while (*str != '\0') {
        lcd_data((uint8_t)*str);
        str++;
    }
}

void LCD_PrintAt(uint8_t row, uint8_t col, const char *str) {
    LCD_SetCursor(row, col);
    LCD_Print(str);
}

void LCD_PrintInt(int32_t value) {
    if (!lcd_initialized) return;

    char buffer[12];  /* Enough for -2147483648 */
    snprintf(buffer, sizeof(buffer), "%ld", (long)value);
    LCD_Print(buffer);
}

void LCD_DisplayOn(void) {
    if (!lcd_initialized) return;
    lcd_display_ctrl |= LCD_DISPLAY_ON;
    lcd_command(LCD_CMD_DISPLAY_CTRL | lcd_display_ctrl);
}

void LCD_DisplayOff(void) {
    if (!lcd_initialized) return;
    lcd_display_ctrl &= ~LCD_DISPLAY_ON;
    lcd_command(LCD_CMD_DISPLAY_CTRL | lcd_display_ctrl);
}

void LCD_BacklightOn(void) {
    lcd_backlight = LCD_PIN_BL;
    lcd_write_i2c(0x00);  /* Update backlight state */
}

void LCD_BacklightOff(void) {
    lcd_backlight = 0;
    lcd_write_i2c(0x00);  /* Update backlight state */
}

void LCD_CursorOn(void) {
    if (!lcd_initialized) return;
    lcd_display_ctrl |= LCD_CURSOR_ON;
    lcd_command(LCD_CMD_DISPLAY_CTRL | lcd_display_ctrl);
}

void LCD_CursorOff(void) {
    if (!lcd_initialized) return;
    lcd_display_ctrl &= ~LCD_CURSOR_ON;
    lcd_command(LCD_CMD_DISPLAY_CTRL | lcd_display_ctrl);
}

void LCD_BlinkOn(void) {
    if (!lcd_initialized) return;
    lcd_display_ctrl |= LCD_BLINK_ON;
    lcd_command(LCD_CMD_DISPLAY_CTRL | lcd_display_ctrl);
}

void LCD_BlinkOff(void) {
    if (!lcd_initialized) return;
    lcd_display_ctrl &= ~LCD_BLINK_ON;
    lcd_command(LCD_CMD_DISPLAY_CTRL | lcd_display_ctrl);
}

void LCD_CreateChar(uint8_t location, const uint8_t *charmap) {
    if (!lcd_initialized || charmap == NULL) return;

    location &= 0x07;  /* Only 8 custom characters (0-7) */
    lcd_command(LCD_CMD_SET_CGRAM | (location << 3));

    for (uint8_t i = 0; i < 8u; i++) {
        lcd_data(charmap[i]);
    }

    /* Return to DDRAM */
    lcd_command(LCD_CMD_SET_DDRAM);
}

void LCD_ScrollLeft(void) {
    if (!lcd_initialized) return;
    lcd_command(LCD_CMD_CURSOR_SHIFT | 0x08);  /* Display shift left */
}

void LCD_ScrollRight(void) {
    if (!lcd_initialized) return;
    lcd_command(LCD_CMD_CURSOR_SHIFT | 0x0C);  /* Display shift right */
}
