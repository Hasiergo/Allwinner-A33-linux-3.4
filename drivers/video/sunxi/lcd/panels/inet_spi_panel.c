#include "panels.h"
#include "inet_spi_panel.h"
#include <mach/sys_config.h>//gongpiqiang+++
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/regulator/consumer.h>

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
	LCD_OPEN_FUNC(sel, LCD_power_on, 30);   //open lcd power, and delay 50ms
	LCD_OPEN_FUNC(sel, LCD_panel_init, 50);   //open lcd power, than delay 200ms
	LCD_OPEN_FUNC(sel, sunxi_lcd_tcon_enable, 100);     //open lcd controller, and delay 100ms
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

static void SPI_3W_SET_CMD(__u8 tx)
{
	__u8 i;

	spi_csx_set(0);
	spi_sdi_set(0);
	sunxi_lcd_delay_us(1);
	spi_sck_set(0);
	sunxi_lcd_delay_us(1);
	spi_sck_set(1);
	sunxi_lcd_delay_us(1);

	
	for(i=0;i<8;i++)
	{
		if(tx & 0x80)
			spi_sdi_set(1);
		else
			spi_sdi_set(0);
		sunxi_lcd_delay_us(1);
		spi_sck_set(0);
		sunxi_lcd_delay_us(1);
		spi_sck_set(1);
		sunxi_lcd_delay_us(1);
		tx <<= 1;
	}
	spi_csx_set(1);
	sunxi_lcd_delay_us(3);
}

static void SPI_3W_SET_PAs(__u8 tx)
{
	__u8 i;

	spi_csx_set(0);
	spi_sdi_set(1);
	sunxi_lcd_delay_us(1);
	spi_sck_set(0);
	sunxi_lcd_delay_us(1);
	spi_sck_set(1);
	sunxi_lcd_delay_us(1);

	
	for(i=0;i<8;i++)
	{
		if(tx & 0x80)
			spi_sdi_set(1);
		else
			spi_sdi_set(0);
		sunxi_lcd_delay_us(1);
		spi_sck_set(0);
		sunxi_lcd_delay_us(1);
		spi_sck_set(1);
		sunxi_lcd_delay_us(1);
		tx <<= 1;
	}
	spi_csx_set(1);
	sunxi_lcd_delay_us(3);
}
/*
static __u8 SPI_3W_GET_PAs(__u8 tx)
{
	__u8 i;
	__u8 read_bit,read_data=0;
	spi_csx_set(0);
	spi_sdi_set(1);
	sunxi_lcd_delay_us(1);
	spi_sck_set(0);
	sunxi_lcd_delay_us(1);
	spi_sck_set(1);
	sunxi_lcd_delay_us(1);

	
	for(i=0;i<8;i++)
	{
		if(tx & 0x80)
			spi_sdi_set(1);
		else
			spi_sdi_set(0);
		sunxi_lcd_delay_us(1);
		spi_sck_set(0);
		sunxi_lcd_delay_us(1);
		spi_sck_set(1);
		sunxi_lcd_delay_us(1);
		tx <<= 1;
	}
	
	for(i=0;i<8;i++)
	{
		read_data <<= 1;
		spi_sck_set(0);
		sunxi_lcd_delay_us(1);
		spi_sdi_get(read_bit);
		printk("read bit1:%d\n",LCD_GPIO_read(0,1));
		if(read_bit)
			read_data |=  0x01;
		else
			read_data &= ~0x01;
		spi_sck_set(1);
		sunxi_lcd_delay_us(1);
		printk("read bit2:%d\n",LCD_GPIO_read(0,1));
	}
	
	return read_data;
}
*/
static void TFT043B027_init(void)
{
	printk("%s\n",__func__);
	int ret;
	static first = 0;
	script_item_value_type_e type;
	script_item_u val;
	script_item_u adc_rst_gpio;
	struct regulator *ldo = NULL;
	if(first) {
	type = script_get_item("spi_keys", "spi_keys_power", &val);
    if(type == SCIRPT_ITEM_VALUE_TYPE_STR){
        printk("spi_keys_power:%s !\n", val.str);
        ldo = regulator_get(NULL, val.str);
        if (ldo) {
            regulator_set_voltage(ldo, 3000000, 3000000);
            ret = regulator_enable(ldo);
            if(ret < 0){
                printk("spi_keys_power regulator_enable failed.\n");
            }
            regulator_put(ldo);
        } else {
            printk("get spi_keys_power regulator failed.\n");
        }
    }
    first = 1;
  }
  
	//panel_rst(1);
	//sunxi_lcd_delay_ms(50);
	//panel_rst(0);

	sunxi_lcd_delay_ms(10);
	panel_rst(1);
	sunxi_lcd_delay_ms(1);
	
	//printk("read_id1:0x%x\n",SPI_3W_GET_PAs(0xda));
	//printk("read_id1:0x%x\n",SPI_3W_GET_PAs(0xdb));
	//printk("read_id1:0x%x\n",SPI_3W_GET_PAs(0xdc));

	#if 0//HX8369	
	SPI_3W_SET_CMD(0xB9);	   //Pass words
	SPI_3W_SET_PAs(0xFF);
	SPI_3W_SET_PAs(0x83);
	SPI_3W_SET_PAs(0x69);
	sunxi_lcd_delay_ms(5);
	
	//SPI_3W_SET_CMD(0xBA);	//SPI+RGBÊ±ÆÁ±Î´Ë¼Ä´æÆ÷ÉèÖÃ
	//SPI_3W_SET_PAs(0x31);   //MIPI_DSI Two Lane //SPI_3W_SET_PAs(0x30); //MIPI_DSI One Lane
	//SPI_3W_SET_PAs(0x00);    
	//SPI_3W_SET_PAs(0x16);    
	//SPI_3W_SET_PAs(0xCA);    
	//SPI_3W_SET_PAs(0xB1);    
	//SPI_3W_SET_PAs(0x0A);    
	//SPI_3W_SET_PAs(0x00);    
	//SPI_3W_SET_PAs(0x10);    
	//SPI_3W_SET_PAs(0x28);    
	//SPI_3W_SET_PAs(0x02);    
	//SPI_3W_SET_PAs(0x21);    
	//SPI_3W_SET_PAs(0x21);    
	//SPI_3W_SET_PAs(0x9A);    
	//SPI_3W_SET_PAs(0x1A);    
	//SPI_3W_SET_PAs(0x8F);  
	
	SPI_3W_SET_CMD(0x3A);	//Interface pixel format
	SPI_3W_SET_PAs(0x70);  
	
	SPI_3W_SET_CMD(0xD5);	//GIP CONTROL
	SPI_3W_SET_PAs(0x00);
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x08);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x0A);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x10);    
	SPI_3W_SET_PAs(0x01);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x01);    
	SPI_3W_SET_PAs(0x49);    
	SPI_3W_SET_PAs(0x37);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x0A);    
	SPI_3W_SET_PAs(0x0A);    
	SPI_3W_SET_PAs(0x0B);    
	SPI_3W_SET_PAs(0x47);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x60);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x03);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x26);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x91);    
	SPI_3W_SET_PAs(0x13);    
	SPI_3W_SET_PAs(0x35);    
	SPI_3W_SET_PAs(0x57);    
	SPI_3W_SET_PAs(0x75);    
	SPI_3W_SET_PAs(0x18);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x86);    
	SPI_3W_SET_PAs(0x64);    
	SPI_3W_SET_PAs(0x42);    
	SPI_3W_SET_PAs(0x20);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x49);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x90);    
	SPI_3W_SET_PAs(0x02);    
	SPI_3W_SET_PAs(0x24);    
	SPI_3W_SET_PAs(0x46);    
	SPI_3W_SET_PAs(0x64);    
	SPI_3W_SET_PAs(0x08);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x87);    
	SPI_3W_SET_PAs(0x75);    
	SPI_3W_SET_PAs(0x53);    
	SPI_3W_SET_PAs(0x31);    
	SPI_3W_SET_PAs(0x11);    
	SPI_3W_SET_PAs(0x59);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x01);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x0F);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x0F);    
	SPI_3W_SET_PAs(0xFF);    
	SPI_3W_SET_PAs(0xFF);    
	SPI_3W_SET_PAs(0x0F);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x0F);    
	SPI_3W_SET_PAs(0xFF);    
	SPI_3W_SET_PAs(0xFF);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x80);    
	SPI_3W_SET_PAs(0x5A);   
	
	SPI_3W_SET_CMD(0xB1);	//Power Control
	SPI_3W_SET_PAs(0x0C);//0c,0x13 
	SPI_3W_SET_PAs(0x83);    
	SPI_3W_SET_PAs(0x77);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x0F);    
	SPI_3W_SET_PAs(0x0F);    
	SPI_3W_SET_PAs(0x18);    
	SPI_3W_SET_PAs(0x18);    
	SPI_3W_SET_PAs(0x0C);    
	SPI_3W_SET_PAs(0x02);  
	
	SPI_3W_SET_CMD(0xB2);	
	SPI_3W_SET_PAs(0x00);  
	SPI_3W_SET_PAs(0x70); 
	 	
	SPI_3W_SET_CMD(0xB3);	 //RGB Setting
	SPI_3W_SET_PAs(0x83);  	   
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x31);    
	SPI_3W_SET_PAs(0x03);    
		
	SPI_3W_SET_CMD(0xB4);	 
	SPI_3W_SET_PAs(0x00);	 //02//two_Dot invertion   0x00 Column_invertion
		
	SPI_3W_SET_CMD(0xB6);	 //Flicker Set  VCOM Voltage
	SPI_3W_SET_PAs(0xA0);//a0	
	SPI_3W_SET_PAs(0xA0);//a0    	
	
	SPI_3W_SET_CMD(0xCB);	
	SPI_3W_SET_PAs(0x6D);	
		
	SPI_3W_SET_CMD(0xCC);	// BGR and GS SS Display direction 
	SPI_3W_SET_PAs(0x02);//06->04->c->e	
	
	SPI_3W_SET_CMD(0xC6);	 //Internal used
	SPI_3W_SET_PAs(0x41);
	SPI_3W_SET_PAs(0xFF);    
	SPI_3W_SET_PAs(0x7A);  
	
	SPI_3W_SET_CMD(0xEA);	//CABC Control
	SPI_3W_SET_PAs(0x72);
	
	SPI_3W_SET_CMD(0xE3);	 //EQ
	SPI_3W_SET_PAs(0x07);
	SPI_3W_SET_PAs(0x0F);      
	SPI_3W_SET_PAs(0x07);    
	SPI_3W_SET_PAs(0x0F);    
	 
	SPI_3W_SET_CMD(0xC0);	   //Set Source Option
	SPI_3W_SET_PAs(0x73);
	SPI_3W_SET_PAs(0x50);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x34);    
	SPI_3W_SET_PAs(0xC4);    
	SPI_3W_SET_PAs(0x09);   
	
	SPI_3W_SET_CMD(0xC1);	//Set DGC
	SPI_3W_SET_PAs(0x00); //turn off DGC 
	  
	SPI_3W_SET_CMD(0xE0); //Gamma Setting	2.2
	SPI_3W_SET_PAs(0x00);
	SPI_3W_SET_PAs(0x07);    
	SPI_3W_SET_PAs(0x0C);    
	SPI_3W_SET_PAs(0x30);    
	SPI_3W_SET_PAs(0x32);    
	SPI_3W_SET_PAs(0x3F);    
	SPI_3W_SET_PAs(0x1C);    
	SPI_3W_SET_PAs(0x3A);    
	SPI_3W_SET_PAs(0x08);    
	SPI_3W_SET_PAs(0x0D);    
	SPI_3W_SET_PAs(0x10);    
	SPI_3W_SET_PAs(0x14);    
	SPI_3W_SET_PAs(0x16);    
	SPI_3W_SET_PAs(0x14);    
	SPI_3W_SET_PAs(0x15);    
	SPI_3W_SET_PAs(0x0E);    
	SPI_3W_SET_PAs(0x12);    
	SPI_3W_SET_PAs(0x00);    
	SPI_3W_SET_PAs(0x07);    
	SPI_3W_SET_PAs(0x0C);    
	SPI_3W_SET_PAs(0x30);    
	SPI_3W_SET_PAs(0x32);    
	SPI_3W_SET_PAs(0x3F);    
	SPI_3W_SET_PAs(0x1C);    
	SPI_3W_SET_PAs(0x3A);    
	SPI_3W_SET_PAs(0x08);    
	SPI_3W_SET_PAs(0x0D);    
	SPI_3W_SET_PAs(0x10);    
	SPI_3W_SET_PAs(0x14);    
	SPI_3W_SET_PAs(0x16);    
	SPI_3W_SET_PAs(0x14);    
	SPI_3W_SET_PAs(0x15);    
	SPI_3W_SET_PAs(0x0E);    
	SPI_3W_SET_PAs(0x12);    
	SPI_3W_SET_PAs(0x01);    
	
	SPI_3W_SET_CMD(0x11); //Sleep Out
	sunxi_lcd_delay_ms(150);
	
	SPI_3W_SET_CMD(0x29); //Display on
	sunxi_lcd_delay_ms(100);
	#else
	SPI_3W_SET_CMD(0xFF);        // Change to Page 1 CMD 
	SPI_3W_SET_PAs(0xFF); 
	SPI_3W_SET_PAs(0x98); 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_PAs(0x04); 
	SPI_3W_SET_PAs(0x01); 
	 
	SPI_3W_SET_CMD(0x08);        //Output    SDA 
	SPI_3W_SET_PAs(0x10); 
	 
	SPI_3W_SET_CMD(0x21);        //DE = 1 Active 
	SPI_3W_SET_PAs(0x01); 
	 
	SPI_3W_SET_CMD(0x30);        //Resolution setting 480 X 800 
	SPI_3W_SET_PAs(0x02); 
	 
	SPI_3W_SET_CMD(0x31);        //Inversion setting 2-dot 
	SPI_3W_SET_PAs(0x02); 
	 
	SPI_3W_SET_CMD(0x40);        //BT  
	SPI_3W_SET_PAs(0x16); 
	 
	SPI_3W_SET_CMD(0x41);        //Default 
	SPI_3W_SET_PAs(0x33); 
	 
	SPI_3W_SET_CMD(0x42);        // 
	SPI_3W_SET_PAs(0x02); 
	 
	SPI_3W_SET_CMD(0x43);        //Default 
	SPI_3W_SET_PAs(0x89); 
	 
	SPI_3W_SET_CMD(0x44);        // 
	SPI_3W_SET_PAs(0x06); 
	 
	SPI_3W_SET_CMD(0x50);        //VREG1 
	SPI_3W_SET_PAs(0x80); 
	 
	SPI_3W_SET_CMD(0x51);        //VREG2 
	SPI_3W_SET_PAs(0x80); 
	 
	SPI_3W_SET_CMD(0x52);        //Flicker MSB 
	SPI_3W_SET_PAs(0x00); 
	 
	SPI_3W_SET_CMD(0x53);        //Flicker LSB 
	SPI_3W_SET_PAs(0x5e);//58 - > 5c   ->5e
	 
	SPI_3W_SET_CMD(0x60);        // 
	SPI_3W_SET_PAs(0x07); 
	 
	SPI_3W_SET_CMD(0x61);        // 
	SPI_3W_SET_PAs(0x00); 
	 
	SPI_3W_SET_CMD(0x62);        // 
	SPI_3W_SET_PAs(0x07); 
	 
	SPI_3W_SET_CMD(0x63);        // 
	SPI_3W_SET_PAs(0x00); 
	 
	 
	SPI_3W_SET_CMD(0xA0);        //Positive Gamma 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0xA1);        // 
	SPI_3W_SET_PAs(0x04); 
	SPI_3W_SET_CMD(0xA2);        // 
	SPI_3W_SET_PAs(0x0C); 
	SPI_3W_SET_CMD(0xA3);        // 
	SPI_3W_SET_PAs(0x0E); 
	SPI_3W_SET_CMD(0xA4);        // 
	SPI_3W_SET_PAs(0x0A); 
	SPI_3W_SET_CMD(0xA5);        // 
	SPI_3W_SET_PAs(0x1C); 
	SPI_3W_SET_CMD(0xA6);        // 
	SPI_3W_SET_PAs(0x0C); 
	SPI_3W_SET_CMD(0xA7);        // 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_CMD(0xA8);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0xA9);        // 
	SPI_3W_SET_PAs(0x0D); 
	SPI_3W_SET_CMD(0xAA);        // 
	SPI_3W_SET_PAs(0x02); 
	SPI_3W_SET_CMD(0xAB);        // 
	SPI_3W_SET_PAs(0x05); 
	SPI_3W_SET_CMD(0xAC);        // 
	SPI_3W_SET_PAs(0x11); 
	SPI_3W_SET_CMD(0xAD);        // 
	SPI_3W_SET_PAs(0x2E); 
	SPI_3W_SET_CMD(0xAE);        // 
	SPI_3W_SET_PAs(0x2B); 
	SPI_3W_SET_CMD(0xAF);        // 
	SPI_3W_SET_PAs(0x00); 
	 
	SPI_3W_SET_CMD(0xC0);        //Negative Gamma 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0xC1);        // 
	SPI_3W_SET_PAs(0x01); 
	SPI_3W_SET_CMD(0xC2);        // 
	SPI_3W_SET_PAs(0x07); 
	SPI_3W_SET_CMD(0xC3);        // 
	SPI_3W_SET_PAs(0x0E); 
	SPI_3W_SET_CMD(0xC4);        // 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_CMD(0xC5);        // 
	SPI_3W_SET_PAs(0x17); 
	SPI_3W_SET_CMD(0xC6);        // 
	SPI_3W_SET_PAs(0x07); 
	SPI_3W_SET_CMD(0xC7);        // 
	SPI_3W_SET_PAs(0x0C); 
	SPI_3W_SET_CMD(0xC8);        // 
	SPI_3W_SET_PAs(0x05); 
	SPI_3W_SET_CMD(0xC9);        // 
	SPI_3W_SET_PAs(0x04); 
	SPI_3W_SET_CMD(0xCA);        // 
	SPI_3W_SET_PAs(0x07); 
	SPI_3W_SET_CMD(0xCB);        // 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_CMD(0xCC);        // 
	SPI_3W_SET_PAs(0x0A); 
	SPI_3W_SET_CMD(0xCD);        // 
	SPI_3W_SET_PAs(0x2B); 
	SPI_3W_SET_CMD(0xCE);        // 
	SPI_3W_SET_PAs(0x24); 
	SPI_3W_SET_CMD(0xCF);        // 
	SPI_3W_SET_PAs(0x00); 
	 
	SPI_3W_SET_CMD(0xFF);        // Change to Page 6 CMD for GIP timing   
	SPI_3W_SET_PAs(0xFF); 
	SPI_3W_SET_PAs(0x98); 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_PAs(0x04); 
	SPI_3W_SET_PAs(0x06); 
	 
	SPI_3W_SET_CMD(0x00);        // 
	SPI_3W_SET_PAs(0x21); 
	SPI_3W_SET_CMD(0x01);        // 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_CMD(0x02);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x03);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x04);        // 
	SPI_3W_SET_PAs(0x01); 
	SPI_3W_SET_CMD(0x05);        // 
	SPI_3W_SET_PAs(0x01); 
	SPI_3W_SET_CMD(0x06);        // 
	SPI_3W_SET_PAs(0x80); 
	SPI_3W_SET_CMD(0x07);        // 
	SPI_3W_SET_PAs(0x02); 
	SPI_3W_SET_CMD(0x08);        // 
	SPI_3W_SET_PAs(0x05); 
	SPI_3W_SET_CMD(0x09);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x0A);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x0B);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x0C);        // 
	SPI_3W_SET_PAs(0x01); 
	SPI_3W_SET_CMD(0x0D);        // 
	SPI_3W_SET_PAs(0x01); 
	SPI_3W_SET_CMD(0x0E);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x0F);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x10);        // 
	SPI_3W_SET_PAs(0xF0); 
	SPI_3W_SET_CMD(0x11);        // 
	SPI_3W_SET_PAs(0xF4); 
	SPI_3W_SET_CMD(0x12);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x13);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x14);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x15);        // 
	SPI_3W_SET_PAs(0xC0); 
	SPI_3W_SET_CMD(0x16);        // 
	SPI_3W_SET_PAs(0x08); 
	SPI_3W_SET_CMD(0x17);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x18);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x19);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x1A);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x1B);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x1C);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x1D);        // 
	SPI_3W_SET_PAs(0x00); 
	SPI_3W_SET_CMD(0x20);        // 
	SPI_3W_SET_PAs(0x02); 
	SPI_3W_SET_CMD(0x21);        // 
	SPI_3W_SET_PAs(0x13); 
	SPI_3W_SET_CMD(0x22);        // 
	SPI_3W_SET_PAs(0x45); 
	SPI_3W_SET_CMD(0x23);        // 
	SPI_3W_SET_PAs(0x67); 
	SPI_3W_SET_CMD(0x24);        // 
	SPI_3W_SET_PAs(0x01); 
	SPI_3W_SET_CMD(0x25);        // 
	SPI_3W_SET_PAs(0x23); 
	SPI_3W_SET_CMD(0x26);        // 
	SPI_3W_SET_PAs(0x45); 
	SPI_3W_SET_CMD(0x27);        // 
	SPI_3W_SET_PAs(0x67); 
	SPI_3W_SET_CMD(0x30);        // 
	SPI_3W_SET_PAs(0x13); 
	SPI_3W_SET_CMD(0x31);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x32);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x33);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x34);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x35);        // 
	SPI_3W_SET_PAs(0xBB); 
	SPI_3W_SET_CMD(0x36);        // 
	SPI_3W_SET_PAs(0xAA); 
	SPI_3W_SET_CMD(0x37);        // 
	SPI_3W_SET_PAs(0xDD); 
	SPI_3W_SET_CMD(0x38);        // 
	SPI_3W_SET_PAs(0xCC); 
	SPI_3W_SET_CMD(0x39);        // 
	SPI_3W_SET_PAs(0x66); 
	SPI_3W_SET_CMD(0x3A);        // 
	SPI_3W_SET_PAs(0x77); 
	SPI_3W_SET_CMD(0x3B);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x3C);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x3D);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x3E);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x3F);        // 
	SPI_3W_SET_PAs(0x22); 
	SPI_3W_SET_CMD(0x40);        // 
	SPI_3W_SET_PAs(0x22); 
	
	SPI_3W_SET_CMD(0xFF);        // Change to Page 7 CMD for Normal command 
	SPI_3W_SET_PAs(0xFF); 
	SPI_3W_SET_PAs(0x98); 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_PAs(0x04); 
	SPI_3W_SET_PAs(0x07); 
	 
	SPI_3W_SET_CMD(0x18); 
	SPI_3W_SET_PAs(0x1D); 
	SPI_3W_SET_CMD(0x26); 
	SPI_3W_SET_PAs(0x92); 
	SPI_3W_SET_CMD(0x02); 
	SPI_3W_SET_PAs(0x70); 
	
	SPI_3W_SET_CMD(0xFF);        // Change to Page 0 CMD for Normal command 
	SPI_3W_SET_PAs(0xFF); 
	SPI_3W_SET_PAs(0x98); 
	SPI_3W_SET_PAs(0x06); 
	SPI_3W_SET_PAs(0x04); 
	SPI_3W_SET_PAs(0x00); 
	
	SPI_3W_SET_CMD(0x3a); //Interface pixel format
	SPI_3W_SET_PAs(0x70); //gongpiqiang,need to set 18bit??
	 
	SPI_3W_SET_CMD(0x11);         //Exit Sleep 
	sunxi_lcd_delay_ms(120); 
	SPI_3W_SET_CMD(0x29);         // Display On 
	#endif
}

static void TFT043B027_exit(void)
{
	printk("%s\n",__func__);
	panel_rst(0);
}

static void LCD_panel_init(u32 sel)
{
	static int first =0;
	script_item_u   val;
  script_item_value_type_e	type;
  printk("inet_spi_panel init\n");
  type = script_get_item("lcd0_para","lcd_model_name", &val);
  if (SCIRPT_ITEM_VALUE_TYPE_STR != type) {  	
	  printk("fetch lcd_model_name from sys_config failed\n");
	} else {
		printk("lcd_model_name = %s\n",val.str);
	}
	if (first == 0) {
		sunxi_lcd_delay_ms(300);
		first =1;
		//printk("delay when first in LCD_panel_init\n");
		}
	if(!strcmp("TFT043B027",val.str))
		TFT043B027_init();
	
	return;
}

static void LCD_panel_exit(u32 sel)
{
	script_item_u   val;
  script_item_value_type_e	type;
  type = script_get_item("lcd0_para","lcd_model_name", &val);
  if (SCIRPT_ITEM_VALUE_TYPE_STR != type) {  	
	  printk("fetch lcd_model_name from sys_config failed\n");
	} else {
		printk("lcd_model_name = %s\n",val.str);
	}
	if(!strcmp("TFT043B027",val.str))
		TFT043B027_exit();
		
	return ;
}

//sel: 0:lcd0; 1:lcd1
static s32 LCD_user_defined_func(u32 sel, u32 para1, u32 para2, u32 para3)
{
	return 0;
}

__lcd_panel_t inet_spi_panel = {
	/* panel driver name, must mach the name of lcd_drv_name in sys_config.fex */
	.name = "inet_spi_panel",
	.func = {
		.cfg_panel_info = LCD_cfg_panel_info,
		.cfg_open_flow = LCD_open_flow,
		.cfg_close_flow = LCD_close_flow,
		.lcd_user_defined_func = LCD_user_defined_func,
	},
};
