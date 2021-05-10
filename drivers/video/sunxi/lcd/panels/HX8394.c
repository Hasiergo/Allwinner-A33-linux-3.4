#include "HX8394.h"
#include "panels.h"

static void LCD_power_on(u32 sel);
static void LCD_power_off(u32 sel);
static void LCD_bl_open(u32 sel);
static void LCD_bl_close(u32 sel);

static void LCD_panel_init(u32 sel);
static void LCD_panel_exit(u32 sel);

static void LCD_cfg_panel_info(panel_extend_para * info)
{
	u32 i = 0, j=0;
	u32 items;
	u8 lcd_gamma_tbl[][2] =
	{
		//{input value, corrected value}
		{0, 0},
		{15, 15},
		{30, 30},
		{45, 45},
		{60, 60},
		{75, 75},
		{90, 90},
		{105, 105},
		{120, 120},
		{135, 135},
		{150, 150},
		{165, 165},
		{180, 180},
		{195, 195},
		{210, 210},
		{225, 225},
		{240, 240},
		{255, 255},
	};

//	u8 lcd_bright_curve_tbl[][2] =
//	{
//		//{input value, corrected value}
//		{0    ,0  },//0
//		{15   ,3  },//0
//		{30   ,6  },//0
//		{45   ,9  },// 1
//		{60   ,12  },// 2
//		{75   ,16  },// 5
//		{90   ,22  },//9
//		{105   ,28 }, //15
//		{120  ,36 },//23
//		{135  ,44 },//33
//		{150  ,54 },
//		{165  ,67 },
//		{180  ,84 },
//		{195  ,108},
//		{210  ,137},
//		{225 ,171},
//		{240 ,210},
//		{255 ,255},
//	};

	u32 lcd_cmap_tbl[2][3][4] = {
	{
		{LCD_CMAP_G0,LCD_CMAP_B1,LCD_CMAP_G2,LCD_CMAP_B3},
		{LCD_CMAP_B0,LCD_CMAP_R1,LCD_CMAP_B2,LCD_CMAP_R3},
		{LCD_CMAP_R0,LCD_CMAP_G1,LCD_CMAP_R2,LCD_CMAP_G3},
		},
		{
		{LCD_CMAP_B3,LCD_CMAP_G2,LCD_CMAP_B1,LCD_CMAP_G0},
		{LCD_CMAP_R3,LCD_CMAP_B2,LCD_CMAP_R1,LCD_CMAP_B0},
		{LCD_CMAP_G3,LCD_CMAP_R2,LCD_CMAP_G1,LCD_CMAP_R0},
		},
	};

	//memset(info,0,sizeof(panel_extend_para));

	items = sizeof(lcd_gamma_tbl)/2;
	for(i=0; i<items-1; i++) {
		u32 num = lcd_gamma_tbl[i+1][0] - lcd_gamma_tbl[i][0];

		for(j=0; j<num; j++) {
			u32 value = 0;

			value = lcd_gamma_tbl[i][1] + ((lcd_gamma_tbl[i+1][1] - lcd_gamma_tbl[i][1]) * j)/num;
			info->lcd_gamma_tbl[lcd_gamma_tbl[i][0] + j] = (value<<16) + (value<<8) + value;
		}
	}
	info->lcd_gamma_tbl[255] = (lcd_gamma_tbl[items-1][1]<<16) + (lcd_gamma_tbl[items-1][1]<<8) + lcd_gamma_tbl[items-1][1];

//	items = sizeof(lcd_bright_curve_tbl)/2;
//	for(i=0; i<items-1; i++) {
//		u32 num = lcd_bright_curve_tbl[i+1][0] - lcd_bright_curve_tbl[i][0];
//
//		for(j=0; j<num; j++) {
//			u32 value = 0;
//
//			value = lcd_bright_curve_tbl[i][1] + ((lcd_bright_curve_tbl[i+1][1] - lcd_bright_curve_tbl[i][1]) * j)/num;
//			info->lcd_bright_curve_tbl[lcd_bright_curve_tbl[i][0] + j] = value;
//		}
//	}
//	info->lcd_bright_curve_tbl[255] = lcd_bright_curve_tbl[items-1][1];

	memcpy(info->lcd_cmap_tbl, lcd_cmap_tbl, sizeof(lcd_cmap_tbl));

}

static s32 LCD_open_flow(u32 sel)
{
	LCD_OPEN_FUNC(sel, LCD_power_on, 100);   //open lcd power, and delay 50ms
	LCD_OPEN_FUNC(sel, LCD_panel_init, 200);   //open lcd power, than delay 200ms
	LCD_OPEN_FUNC(sel, sunxi_lcd_tcon_enable, 200);     //open lcd controller, and delay 100ms
	LCD_OPEN_FUNC(sel, LCD_bl_open, 0);     //open lcd backlight, and delay 0ms

	return 0;
}

static s32 LCD_close_flow(u32 sel)
{
	LCD_CLOSE_FUNC(sel, LCD_bl_close, 0);       //close lcd backlight, and delay 0ms
	LCD_CLOSE_FUNC(sel, sunxi_lcd_tcon_disable, 0);         //close lcd controller, and delay 0ms
	LCD_CLOSE_FUNC(sel, LCD_panel_exit,	200);   //open lcd power, than delay 200ms
	LCD_CLOSE_FUNC(sel, LCD_power_off, 500);   //close lcd power, and delay 500ms

	return 0;
}

static void LCD_power_on(u32 sel)
{
	sunxi_lcd_power_enable(sel, 0);//config lcd_power pin to open lcd power0
	sunxi_lcd_delay_ms(300);//足赤?車?車那㊣㏒?豕﹞㊣㏒那1車?車2?t?∩??米??﹞米?～?㏒??迆谷?米?o車㏒??∩??D?o??y3㏒o車?迄3?那??‘㏒?2⊿?赤tcon那y?Y
	sunxi_lcd_pin_cfg(sel, 1);
}

static void LCD_power_off(u32 sel)
{
	sunxi_lcd_pin_cfg(sel, 0);
	sunxi_lcd_power_disable(sel, 0);//config lcd_power pin to close lcd power0
}

static void LCD_bl_open(u32 sel)
{
	sunxi_lcd_pwm_enable(sel);//open pwm module
	sunxi_lcd_backlight_enable(sel);//config lcd_bl_en pin to open lcd backlight
}

static void LCD_bl_close(u32 sel)
{
	sunxi_lcd_backlight_disable(sel);//config lcd_bl_en pin to close lcd backlight
	sunxi_lcd_pwm_disable(sel);//close pwm module
}

struct lcd_setting_table {
    u8 cmd;
    u32 count;
    u8 para[64];
};

//new 20140411
static struct lcd_setting_table lcd_initialization_setting[] = {
		{0xB9,3,{0xFF,0x83,0x94}},
		{0xB1,15,{0x64,0x0C,0x2C,0x44,0x34,0x11,0xF1,0x81,0x28,0x99,0x34,0x80,0xC0,0xD2,0x01}},
		{0xB2,12,{0x45,	0x44,	0x0F,	0x09,	0x81,	0x1C,	0x08,	0x08,	0x1C,	0x4D,	0x00,	0x00}},
		{0xB4,22,{0x00,	0xFF,	0x58,	0x58,	0x58,	0x48,	0x00,	0x00,	0x01,	0x72,	0x0F,	0x72,	0x58,	0x58,	0x58,	0x48,	0x00,	0x00,	0x01,	0x72,	0x0F,	0x72}},
		{0xB6,2,{0x5A,0x5A}},
		{0xCC,1,{0x09}},
		{0xD3,37,{0x00,0x08,0x00,0x01,0x07,0x00,0x08,0x32,0x10,0x0A,0x00,0x05,0x00,0x20,0x0A,0x05,0x09,0x00,0x32,0x10,0x08,0x00,0x35,0x33,0x0D,0x07,0x47,0x0D,0x07,0x47,0x0F,0x08,0x00,0x00,0x0A,0x00,0x00}},
		{0xD5,44,{0x03, 0x02, 0x03, 0x02, 0x01, 0x00, 0x01, 0x00, 0x07, 0x06, 0x07, 0x06, 0x05, 0x04, 0x05, 0x04, 0x21, 0x20, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x23, 0x22, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18}},
		{0xE0,42,{0x01, 0x04, 0x07, 0x0C, 0x0B, 0x11, 0x24, 0x34, 0x08, 0x0D, 0x0F, 0x19, 0x11, 0x16, 0x19, 0x17, 0x16, 0x08, 0x14, 0x16, 0x19, 0x01, 0x04, 0x07, 0x0C, 0x0B, 0x11, 0x24, 0x34, 0x08, 0x0D, 0x0F, 0x19, 0x11, 0x16, 0x19, 0x17, 0x16, 0x08, 0x14, 0x16, 0x19}},
		{0xC9,5,{0x1F, 0x2E, 0x1E, 0x1E, 0x10}},
		//{0x36,1, {0x82}},
		{0x11,99,{220}},
		{0x29,99,{150}},
};


static void LCD_panel_init(u32 sel)
{
	int i;
	sunxi_lcd_pin_cfg(sel, 1);
	sunxi_lcd_delay_ms(10);

	panel_rst(0);
	sunxi_lcd_delay_ms(10);
	panel_rst(1);
	sunxi_lcd_delay_ms(10);
	
	for(i=0;i<sizeof(lcd_initialization_setting) / sizeof(struct lcd_setting_table);i++)
	{
		if(lcd_initialization_setting[i].count == 99)
		{
			dsi_dcs_wr_0para(0,lcd_initialization_setting[i].cmd);
			sunxi_lcd_delay_ms(lcd_initialization_setting[i].para[0]);			
		}
		else
		{		
			dsi_dcs_wr(0, lcd_initialization_setting[i].cmd, lcd_initialization_setting[i].para, lcd_initialization_setting[i].count);
		}
		sunxi_lcd_delay_ms(10);	
	}
	sunxi_lcd_dsi_clk_enable(sel);
	return;
}

static void LCD_panel_exit(u32 sel)
{
	sunxi_lcd_dsi_clk_disable(sel);
	panel_rst(0);
	return ;
}

//sel: 0:lcd0; 1:lcd1
static s32 LCD_user_defined_func(u32 sel, u32 para1, u32 para2, u32 para3)
{
	return 0;
}

__lcd_panel_t HX8394_panel = {
	/* panel driver name, must mach the name of lcd_drv_name in sys_config.fex */
	.name = "HX8394",
	.func = {
		.cfg_panel_info = LCD_cfg_panel_info,
		.cfg_open_flow = LCD_open_flow,
		.cfg_close_flow = LCD_close_flow,
		.lcd_user_defined_func = LCD_user_defined_func,
	},
};
