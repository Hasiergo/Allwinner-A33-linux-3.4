#ifndef  __INET_SPI_PANEL_H__
#define  __INET_SPI_PANEL_H__

#include "panels.h"

#define spi_csx_set(v)	(sunxi_lcd_gpio_set_value(0, 3, v))
#define spi_sck_set(v)  (sunxi_lcd_gpio_set_value(0, 0, v))
#define spi_sdi_set(v)  (sunxi_lcd_gpio_set_value(0, 1, v))
//#define adc_reset(v)    (sunxi_lcd_gpio_set_value(0, 4, v)) 

#define panel_rst(v)    (sunxi_lcd_gpio_set_value(0, 2, v))
extern __lcd_panel_t inet_spi_panel;

#endif
