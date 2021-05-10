#ifndef  __DEFAULT_PANEL_H__
#define  __DEFAULT_PANEL_H__

#include "panels.h"

#define panel_rst(v)    (sunxi_lcd_gpio_set_value(0, 0, v))
#define panel_standby(v)    (sunxi_lcd_gpio_set_value(0, 1, v))
extern __lcd_panel_t default_panel;

#endif
