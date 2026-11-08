#ifndef __OLED_H__
#define __OLED_H__

// The width of the screen can be set using this
// define. The default value is 128.
#define SSD1306_WIDTH           40

// If your screen horizontal axis does not start
// in column 0 you can use this define to
// adjust the horizontal offset
#define SSD1306_X_OFFSET        0

// The height can be changed as well if necessary.
// It can be 32, 64 or 128. The default value is 64.
#define SSD1306_HEIGHT          72

void oled_update(void);
void oled_write(uint16_t n);
#endif /* __OLED_H__ */