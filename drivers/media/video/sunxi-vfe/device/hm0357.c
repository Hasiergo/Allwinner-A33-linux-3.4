/*
 * A V4L2 driver for GalaxyCore HM0357 cameras.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>


#include "camera.h"


MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for GalaxyCore HM0357 sensors");
MODULE_LICENSE("GPL");



//for internel driver debug
#define DEV_DBG_EN   		0
#if(DEV_DBG_EN == 1)
#define vfe_dev_dbg(x,arg...) printk("[CSI_DEBUG][HM0357]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...)
#endif
#define vfe_dev_err(x,arg...) printk("[CSI_ERR][HM0357]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[CSI][HM0357]"x,##arg)

#define LOG_ERR_RET(x)  { \
                          int ret;  \
                          ret = x; \
                          if(ret < 0) {\
                            vfe_dev_err("error at %s\n",__func__);  \
                            return ret; \
                          } \
                        }

//define module timing
#define MCLK (24*1000*1000)
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR 0x0357

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0

#define regval_list reg_list_a16_d8
#define REG_TERM 0xffff
#define VAL_TERM 0xff
#define REG_DLY  0xffff


/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 10

/*
 * The hm0357 sits on i2c with ID 0x60
 */
#define I2C_ADDR 0x60
#define SENSOR_NAME "hm0357"

/*
 * Information we maintain about a known sensor.
 */
struct sensor_format_struct;  /* coming later */

struct cfg_array { /* coming later */
	struct regval_list * regs;
	int size;
};

static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
  return container_of(sd, struct sensor_info, sd);
}
 
static struct regval_list sensor_default_regs[] = {
	{0x0022,0x00}, //
	{0x0020,0x00}, //
	{0x0023,0x98}, //
	{0x008F,0x00}, //
	{0x0004,0x11}, //
	{0x000F,0x00}, //																									  
	{0x0012,0x09},
	{0x0013,0x03},
	{0x0015,0x02}, //																									  
	{0x0016,0x01}, //																									  
	{0x0025,0x00},
	{0x0027,0x10}, // YCbYCr																							  
	{0x0040,0x0F}, // BLC Ph1																							  
	{0x0053,0x0F}, // BLC Ph2																							  
	{0x0075,0x40}, // Auto Negative Pump																				  
	{0x0041,0x02}, //																									  
	{0x0045,0xCA}, //																									  
	{0x0046,0x4F}, //																									  
	{0x004A,0x0A}, //																									  
	{0x004B,0x72}, //																									  
	{0x004D,0xBF}, //																									  
	{0x004E,0x30}, //																							   
	{0x0055,0x10}, //																									  
	{0x0053,0x00}, //																									  
	{0x0070,0x44}, //																									  
	{0x0071,0xB9}, //																									  
	{0x0072,0x55}, //																									  
	{0x0073,0x10}, //																									  
	{0x0081,0xD2}, //																									  
	{0x0082,0xA6}, //																									  
	{0x0083,0x70}, //																									  
	{0x0085,0x11}, //																									  
	{0x0086,0xA7}, //																									  
	{0x0088,0xE1}, //																									  
	{0x008A,0x2D}, //																									  
	{0x008D,0x20}, //																									  
	{0x0090,0x00}, //																									  
	{0x0091,0x10}, //																									  
	{0x0092,0x11}, //																									  
	{0x0093,0x12}, //																									  
	{0x0094,0x13}, //																									  
	{0x0095,0x17}, //																									  
	{0x00A0,0xC0}, //																									  
	{0x011F,0x44}, //																									  
	{0x0120,0x36},
	{0x0121,0x80}, //																									  
	{0x0122,0x6B}, // SIM_BPC,LSC_ON,A6_ON																				  
	{0x0123,0xA5}, //																									  
	{0x0124,0xD2}, //																									  
	{0x0125,0xDE}, // Disable F2B function																							 
	{0x0126,0x71}, // 70=EV_SEL 3x5,60=EV_SEL 3x3																		  
	{0x0140,0x14}, // BPC																								  
	{0x0141,0x0A}, //																									  
	{0x0142,0x14}, //																									  
	{0x0143,0x0A}, //																									  
	{0x0144,0x0C}, //																									  
	{0x0145,0x03}, //																									  
	{0x0146,0x40}, // New BPC																							  
	{0x0147,0x0A}, //																									  
	{0x0148,0x70}, //																									  
	{0x0149,0x0C}, //																									  
	{0x014A,0x80}, //																									  
	{0x014B,0x80}, //																									  
	{0x014C,0x80}, //																									  
	{0x014D,0x2E}, //																									  
	{0x014E,0x05}, //																									  
	{0x014F,0x05}, //																									  
	{0x0150,0x0D}, //																									  
	{0x0155,0x00}, //																									  
	{0x0156,0x0A}, //																									  
	{0x0157,0x0A}, //																									  
	{0x0158,0x0A}, //																									  
	{0x0159,0x0A}, //																									  
	{0x0160,0x14}, // SIM_BPC_th																						  
	{0x0161,0x14}, // SIM_BPC_th Alpha																					  
	{0x0162,0x0A}, // SIM_BPC_Directional																				  
	{0x0163,0x0A}, // SIM_BPC_Directional Alpha 																		  
	{0x01B0,0x33}, // G1G2 Balance																						  
	{0x01B1,0x10}, //																									  
	{0x01B2,0x10}, //																									  
	{0x01B3,0x30}, //																									  
	{0x01B4,0x18}, //																									  
	{0x01D8,0x50}, // Bayer Denoise 																					  
	{0x01DE,0x70}, //																									  
	{0x01E4,0x08}, //																									  
	{0x01E5,0x08}, //																									  
	{0x0220,0x00}, //LSC																								  
	{0x0221,0xBD}, //																									  
	{0x0222,0x00}, //																									  
	{0x0223,0x80}, //																									  
	{0x0224,0x8A}, //																									  
	{0x0225,0x00}, //																									  
	{0x0226,0x80}, //																									  
	{0x0227,0x8A}, //																									  
	{0x0229,0x80}, //																									  
	{0x022A,0x80}, //																									  
	{0x022B,0x00}, //																									  
	{0x022C,0x80}, //																									  
	{0x022D,0x11}, //																									  
	{0x022E,0x10}, //																									  
	{0x022F,0x11}, //																									  
	{0x0230,0x10}, //																									  
	{0x0231,0x11}, //																									  
	{0x0233,0x11}, //																									  
	{0x0234,0x10}, //																									  
	{0x0235,0x46}, //																									  
	{0x0236,0x01}, //																									  
	{0x0237,0x46}, //																									  
	{0x0238,0x01}, //																									  
	{0x023B,0x46}, //																									  
	{0x023C,0x01}, //																									  
	{0x023D,0xF8}, //																									  
	{0x023E,0x00}, //																									  
	{0x023F,0xF8}, //																									  
	{0x0240,0x00}, //																									  
	{0x0243,0xF8}, //																									  
	{0x0244,0x00}, //																									  
	{0x0250,0x03}, //																									  
	{0x0251,0x0D}, //																									  
	{0x0252,0x08}, //																									  
	{0x0280,0x0B}, //Gamma																								  
	{0x0282,0x15}, //																									  
	{0x0284,0x2A}, //																									  
	{0x0286,0x4C}, //																									  
	{0x0288,0x5A}, //																									  
	{0x028A,0x67}, //																									  
	{0x028C,0x73}, //																									  
	{0x028E,0x7D}, //																									  
	{0x0290,0x86}, //																									  
	{0x0292,0x8E}, //																									  
	{0x0294,0x9E}, //																									  
	{0x0296,0xAC}, //																									  
	{0x0298,0xC0}, //																									  
	{0x029A,0xD2}, //																									  
	{0x029C,0xE2}, //																									  
	{0x029E,0x27}, //																	   
	{0x02A0,0x04}, //																																																			
	{0x02C0,0xCC}, //D CCM																										  
	{0x02C1,0x01}, //																									  
	{0x02C2,0xBB}, //																									  
	{0x02C3,0x04}, //																									  
	{0x02C4,0x11}, //																									  
	{0x02C5,0x04}, //																									  
	{0x02C6,0x2E}, //																									  
	{0x02C7,0x04}, //																									  
	{0x02C8,0x6C}, //																									  
	{0x02C9,0x01}, //																									  
	{0x02CA,0x3E}, //																									  
	{0x02CB,0x04}, //																									  
	{0x02CC,0x04}, //																									  
	{0x02CD,0x04}, //																									  
	{0x02CE,0xBE}, //																									  
	{0x02CF,0x04}, //																									  
	{0x02D0,0xC2}, //																									  
	{0x02D1,0x01}, //																								   
	{0x02F0,0xA3}, // CCM DIFF																							  
	{0x02F1,0x04}, //																									  
	{0x02F2,0xE9}, //																									  
	{0x02F3,0x00}, //																									  
	{0x02F4,0x45}, //																									  
	{0x02F5,0x04}, //																									  
	{0x02F6,0x1D}, //																									  
	{0x02F7,0x04}, //																									  
	{0x02F8,0x45}, //																									  
	{0x02F9,0x04}, //																									  
	{0x02FA,0x63}, //																									  
	{0x02FB,0x00}, //																									  
	{0x02FC,0x3E}, //																									  
	{0x02FD,0x04}, //																									  
	{0x02FE,0x39}, //																									  
	{0x02FF,0x04}, //																									  
	{0x0300,0x79}, //																									  
	{0x0301,0x00}, //																							  
	{0x0328,0x00}, // Win Pick P Min																					  
	{0x0329,0x04}, //																									  
	{0x032D,0x66}, // Initial Channel Gain																				  
	{0x032E,0x01}, //																									  
	{0x032F,0x00}, //																									  
	{0x0330,0x01}, //																									  
	{0x0331,0x66}, //																									  
	{0x0332,0x01}, //																									  
	{0x0333,0x00}, // AWB channel offset																				  
	{0x0334,0x00}, //																									  
	{0x0335,0x00}, //																									  
	{0x033E,0x00}, //																									  
	{0x033F,0x00}, //																									  
	{0x0340,0x38}, // AWB																								  
	{0x0341,0x41}, //																									  
	{0x0342,0x04}, //																									  
	{0x0343,0x2C}, //																									  
	{0x0344,0x80}, //																									  
	{0x0345,0x37}, //																									  
	{0x0346,0x80}, //																									  
	{0x0347,0x3D}, //																									  
	{0x0348,0x76}, //																									  
	{0x0349,0x42}, //																									  
	{0x034A,0x6B}, //																									  
	{0x034B,0x68}, //																									  
	{0x034C,0x58}, //																									  
	{0x0350,0x90}, //																									  
	{0x0351,0x90}, //																									  
	{0x0352,0x18}, //																									  
	{0x0353,0x18}, //																									  
	{0x0354,0x73}, // R Max 																							  
	{0x0355,0x45}, // G Max 																							  
	{0x0356,0x78}, // B Max 																							  
	{0x0357,0xD0}, // R+B Max																							  
	{0x0358,0x05}, // R comp Max																						  
	{0x035A,0x05}, //																									  
	{0x035B,0xA0}, // R+B Min																							  
	{0x0381,0x5c},
	{0x0382,0x44},
	{0x0383,0x20}, //																									  
	{0x038A,0x80}, //																									  
	{0x038B,0x10}, //																									  
	{0x038C,0xD1}, //																									  
	{0x038E,0x50},
	{0x038F,0x07}, //AE Max exposure 10 fps 																					
	{0x0390,0xD0}, //AE Max exposure																					  
	{0x0391,0x7C},
	{0x0393,0x80}, //																									  
	{0x0395,0x12}, //																									  
	{0x0398,0x01}, // Frame rate control																				  
	{0x0399,0xF0}, // 1. 30fps																				 
	{0x039A,0x03}, //																									  
	{0x039B,0x00}, // 2. 20fps																									  
	{0x039C,0x04}, //																									  
	{0x039D,0x00},
	{0x039E,0x06}, //																									  
	{0x039F,0x00}, // 4. 10fps																									  
	{0x03A0,0x09},
	{0x03A1,0x50},
	{0x03A6,0x10}, //																									  
	{0x03A7,0x10}, //																									  
	{0x03A8,0x36}, //																									  
	{0x03A9,0x40}, //																									  
	{0x03AE,0x26}, //																									  
	{0x03AF,0x22}, //																									  
	{0x03B0,0x0A}, //																									  
	{0x03B1,0x08}, //																									  
	{0x03B3,0x00}, //																									  
	{0x03B5,0x08}, //																									  
	{0x03B7,0xA0}, //																									  
	{0x03B9,0xD0}, //																									  
	{0x03BB,0x5F}, // AE Winding																						  
	{0x03BE,0x00}, // AE Winding																						  
	{0x03BF,0x1D}, //																									  
	{0x03C0,0x2E}, //																									  
	{0x03C3,0x0F}, //																									  
	{0x03D0,0xE0}, //																									  
	{0x0420,0x86}, // R Offset																							
	{0x0421,0x00}, //																									  
	{0x0422,0x00}, //																									  
	{0x0423,0x00}, //																									  
	{0x0430,0x00}, // ABLC																							   
	{0x0431,0x60}, //Max																								  
	{0x0432,0x30}, //ABLC Tolerence 																							   
	{0x0433,0x30}, //																									  
	{0x0434,0x00}, //																									  
	{0x0435,0x40}, //																									  
	{0x0436,0x00}, //																									  
	{0x0450,0xFF}, //																									  
	{0x0451,0xFF}, //																									  
	{0x0452,0xB0}, //																									  
	{0x0453,0x70}, //																									  
	{0x0454,0x00}, //																									  
	{0x045A,0x00}, //																									  
	{0x045B,0x10}, //																									  
	{0x045C,0x00}, //																									  
	{0x045D,0xA0}, //																									  
	{0x0465,0x02}, //																									  
	{0x0466,0x04}, //																									  
	{0x047A,0x20}, // ODEL Rayer denoise																				  
	{0x047B,0x20}, // ODEL Y denoise																					  
	{0x0480,0x50},//saturation
	{0x0481,0x06}, //Sat by Alpha																						  
	{0x04B0,0x50},//contrast 58 -> 50 by jink 20160530
	{0x04B1,0x00}, //Contrast																							  
	{0x04B3,0x7F}, //Contrast																							  
	{0x04B4,0x00}, //Contrast																							  
	{0x04B6,0x20}, //Contrast																							  
	{0x04B9,0x40}, //Contrast
	{0x04C0,0x18}, //brightness 20160530																								  
	{0x0540,0x00}, //60Hz Flicker step																					  
	{0x0541,0x78},
	{0x0542,0x00}, //50Hz Flicker step																					  
	{0x0543,0x90},
	{0x0580,0x02},//denoise
	{0x0581,0x06},//low light denoise
	{0x0590,0x20}, // UV denoise																						  
	{0x0591,0x30}, // UV denoise Alpha																					  
	{0x0594,0x02}, // UV Gray TH																						  
	{0x0595,0x08}, // UV Gray TH Alpha																					  
	{0x05A0,0x08}, // Sharpness Curve Edge THL																			  
	{0x05A1,0x0C}, // Edge THL Alpha																					  
	{0x05A2,0x50}, // Edge THH																							  
	{0x05A3,0x60}, // Edge THH Alpha																					  
	{0x05B0,0x20},//sharpness 18->28->20 by jink 20140630
	{0x05B1,0x06}, //																									  
	{0x05D0,0x2D}, // F2B Start Mean																					  
	{0x05D1,0x07}, // F2B																								  
	{0x05E4,0x04}, // Windowing 																						  
	{0x05E5,0x00}, //																									  
	{0x05E6,0x83}, //																									  
	{0x05E7,0x02}, //																									  
	{0x05E8,0x04}, //																									  
	{0x05E9,0x00}, //																									  
	{0x05EA,0xE3}, //																									  
	{0x05EB,0x01}, //
	{0x0000,0x01}, //																									  
	{0x0100,0x01}, //																									  
	{0x0101,0x01}, //																									  
	{0x0005,0x01}, // Turn on rolling shutter 
};


/*
 * The white balance settings
 * Here only tune the R G B channel gain.
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_manual[] = {
//null
};

static struct regval_list sensor_wb_auto_regs[] = {
{0x0380,0xFF},
{0x0000,0x01},
};

static struct regval_list sensor_wb_incandescence_regs[] = {
	//bai re guang
{0x0380,0xFD},
{0x032D,0x18},
{0x032E,0x01},
{0x032F,0x00},
{0x0330,0x01},
{0x0331,0xF0},
{0x0332,0x01},
{0x0000,0x01},
{0x0101,0x01},
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//ri guang deng
{0x0380,0xFD},
{0x032D,0x30},
{0x032E,0x01},
{0x032F,0x00},
{0x0330,0x01},
{0x0331,0xB6},
{0x0332,0x01},
{0x0000,0x01},
{0x0101,0x01},
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng
{0x0380,0xFD},
{0x032D,0x20},
{0x032E,0x01},
{0x032F,0x00},
{0x0330,0x01},
{0x0331,0xD0},
{0x0332,0x01},
{0x0000,0x01},
{0x0101,0x01},
};

static struct regval_list sensor_wb_horizon[] = {
//null
};

static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
{0x0380,0xFD},
{0x032D,0xA0},
{0x032E,0x01},
{0x032F,0x00},
{0x0330,0x01},
{0x0331,0x00},
{0x0332,0x01},
{0x0000,0x01},
{0x0101,0x01},
};

static struct regval_list sensor_wb_flash[] = { 
//null
};


static struct regval_list sensor_wb_cloud_regs[] = {
{0x0380,0xFD},
{0x032D,0xC0},
{0x032E,0x01},
{0x032F,0x00},
{0x0330,0x01},
{0x0331,0x00},
{0x0332,0x01},
{0x0000,0x01},
{0x0101,0x01},
};

static struct regval_list sensor_wb_shade[] = {
//null
};

static struct cfg_array sensor_wb[] = {
  {
  	.regs = sensor_wb_manual,             //V4L2_WHITE_BALANCE_MANUAL
    .size = ARRAY_SIZE(sensor_wb_manual),
  },
  {
  	.regs = sensor_wb_auto_regs,          //V4L2_WHITE_BALANCE_AUTO
    .size = ARRAY_SIZE(sensor_wb_auto_regs),
  },
  {
  	.regs = sensor_wb_incandescence_regs, //V4L2_WHITE_BALANCE_INCANDESCENT
    .size = ARRAY_SIZE(sensor_wb_incandescence_regs),
  },
  {
  	.regs = sensor_wb_fluorescent_regs,   //V4L2_WHITE_BALANCE_FLUORESCENT
    .size = ARRAY_SIZE(sensor_wb_fluorescent_regs),
  },
  {
  	.regs = sensor_wb_tungsten_regs,      //V4L2_WHITE_BALANCE_FLUORESCENT_H
    .size = ARRAY_SIZE(sensor_wb_tungsten_regs),
  },
  {
  	.regs = sensor_wb_horizon,            //V4L2_WHITE_BALANCE_HORIZON
    .size = ARRAY_SIZE(sensor_wb_horizon),
  },
  {
  	.regs = sensor_wb_daylight_regs,      //V4L2_WHITE_BALANCE_DAYLIGHT
    .size = ARRAY_SIZE(sensor_wb_daylight_regs),
  },
  {
  	.regs = sensor_wb_flash,              //V4L2_WHITE_BALANCE_FLASH
    .size = ARRAY_SIZE(sensor_wb_flash),
  },
  {
  	.regs = sensor_wb_cloud_regs,         //V4L2_WHITE_BALANCE_CLOUDY
    .size = ARRAY_SIZE(sensor_wb_cloud_regs),
  },
  {
  	.regs = sensor_wb_shade,              //V4L2_WHITE_BALANCE_SHADE
    .size = ARRAY_SIZE(sensor_wb_shade),
  },
//  {
//  	.regs = NULL,
//    .size = 0,
//  },
};


/*
 * The color effect settings
 */
static struct regval_list sensor_colorfx_none_regs[] = {
{0x0488,0x10},
{0x0000,0x01},
};

static struct regval_list sensor_colorfx_bw_regs[] = {
{0x0486,0x80},
{0x0487,0x80},
{0x0488,0x11},
{0x0000,0x01},
};

static struct regval_list sensor_colorfx_sepia_regs[] = {
{0x0486,0x58},
{0x0487,0x90},
{0x0488,0x11},
{0x0000,0x01},
};

static struct regval_list sensor_colorfx_negative_regs[] = {
{0x0486,0x00},
{0x0487,0xFF},
{0x0488,0x12},
{0x0000,0x01},
};

static struct regval_list sensor_colorfx_emboss_regs[] = {

};

static struct regval_list sensor_colorfx_sketch_regs[] = {

};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {

};

static struct regval_list sensor_colorfx_grass_green_regs[] = {

};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {

};

static struct regval_list sensor_colorfx_vivid_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_aqua_regs[] = {
//null
};

static struct regval_list sensor_colorfx_art_freeze_regs[] = {
//null
};

static struct regval_list sensor_colorfx_silhouette_regs[] = {
//null
};

static struct regval_list sensor_colorfx_solarization_regs[] = {
//null
};

static struct regval_list sensor_colorfx_antique_regs[] = {
//null
};

static struct regval_list sensor_colorfx_set_cbcr_regs[] = {
//null
};

static struct cfg_array sensor_colorfx[] = {
  {
  	.regs = sensor_colorfx_none_regs,         //V4L2_COLORFX_NONE = 0,
    .size = ARRAY_SIZE(sensor_colorfx_none_regs),
  },
  {
  	.regs = sensor_colorfx_bw_regs,           //V4L2_COLORFX_BW   = 1,
    .size = ARRAY_SIZE(sensor_colorfx_bw_regs),
  },
  {
  	.regs = sensor_colorfx_sepia_regs,        //V4L2_COLORFX_SEPIA  = 2,
    .size = ARRAY_SIZE(sensor_colorfx_sepia_regs),
  },
  {
  	.regs = sensor_colorfx_negative_regs,     //V4L2_COLORFX_NEGATIVE = 3,
    .size = ARRAY_SIZE(sensor_colorfx_negative_regs),
  },
  {
  	.regs = sensor_colorfx_emboss_regs,       //V4L2_COLORFX_EMBOSS = 4,
    .size = ARRAY_SIZE(sensor_colorfx_emboss_regs),
  },
  {
  	.regs = sensor_colorfx_sketch_regs,       //V4L2_COLORFX_SKETCH = 5,
    .size = ARRAY_SIZE(sensor_colorfx_sketch_regs),
  },
  {
  	.regs = sensor_colorfx_sky_blue_regs,     //V4L2_COLORFX_SKY_BLUE = 6,
    .size = ARRAY_SIZE(sensor_colorfx_sky_blue_regs),
  },
  {
  	.regs = sensor_colorfx_grass_green_regs,  //V4L2_COLORFX_GRASS_GREEN = 7,
    .size = ARRAY_SIZE(sensor_colorfx_grass_green_regs),
  },
  {
  	.regs = sensor_colorfx_skin_whiten_regs,  //V4L2_COLORFX_SKIN_WHITEN = 8,
    .size = ARRAY_SIZE(sensor_colorfx_skin_whiten_regs),
  },
  {
  	.regs = sensor_colorfx_vivid_regs,        //V4L2_COLORFX_VIVID = 9,
    .size = ARRAY_SIZE(sensor_colorfx_vivid_regs),
  },
  {
  	.regs = sensor_colorfx_aqua_regs,         //V4L2_COLORFX_AQUA = 10,
    .size = ARRAY_SIZE(sensor_colorfx_aqua_regs),
  },
  {
  	.regs = sensor_colorfx_art_freeze_regs,   //V4L2_COLORFX_ART_FREEZE = 11,
    .size = ARRAY_SIZE(sensor_colorfx_art_freeze_regs),
  },
  {
  	.regs = sensor_colorfx_silhouette_regs,   //V4L2_COLORFX_SILHOUETTE = 12,
    .size = ARRAY_SIZE(sensor_colorfx_silhouette_regs),
  },
  {
  	.regs = sensor_colorfx_solarization_regs, //V4L2_COLORFX_SOLARIZATION = 13,
    .size = ARRAY_SIZE(sensor_colorfx_solarization_regs),
  },
  {
  	.regs = sensor_colorfx_antique_regs,      //V4L2_COLORFX_ANTIQUE = 14,
    .size = ARRAY_SIZE(sensor_colorfx_antique_regs),
  },
  {
  	.regs = sensor_colorfx_set_cbcr_regs,     //V4L2_COLORFX_SET_CBCR = 15,
    .size = ARRAY_SIZE(sensor_colorfx_set_cbcr_regs),
  },
};



/*
 * The brightness setttings
 */
static struct regval_list sensor_brightness_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_zero_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_brightness_pos4_regs[] = {
//NULL
};

static struct cfg_array sensor_brightness[] = {
  {
  	.regs = sensor_brightness_neg4_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg4_regs),
  },
  {
  	.regs = sensor_brightness_neg3_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg3_regs),
  },
  {
  	.regs = sensor_brightness_neg2_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg2_regs),
  },
  {
  	.regs = sensor_brightness_neg1_regs,
  	.size = ARRAY_SIZE(sensor_brightness_neg1_regs),
  },
  {
  	.regs = sensor_brightness_zero_regs,
  	.size = ARRAY_SIZE(sensor_brightness_zero_regs),
  },
  {
  	.regs = sensor_brightness_pos1_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos1_regs),
  },
  {
  	.regs = sensor_brightness_pos2_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos2_regs),
  },
  {
  	.regs = sensor_brightness_pos3_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos3_regs),
  },
  {
  	.regs = sensor_brightness_pos4_regs,
  	.size = ARRAY_SIZE(sensor_brightness_pos4_regs),
  },
};

/*
 * The contrast setttings
 */
static struct regval_list sensor_contrast_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_zero_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_contrast_pos4_regs[] = {
};

static struct cfg_array sensor_contrast[] = {
  {
  	.regs = sensor_contrast_neg4_regs,
  	.size = ARRAY_SIZE(sensor_contrast_neg4_regs),
  },
  {
  	.regs = sensor_contrast_neg3_regs,
  	.size = ARRAY_SIZE(sensor_contrast_neg3_regs),
  },
  {
  	.regs = sensor_contrast_neg2_regs,
  	.size = ARRAY_SIZE(sensor_contrast_neg2_regs),
  },
  {
  	.regs = sensor_contrast_neg1_regs,
  	.size = ARRAY_SIZE(sensor_contrast_neg1_regs),
  },
  {
  	.regs = sensor_contrast_zero_regs,
  	.size = ARRAY_SIZE(sensor_contrast_zero_regs),
  },
  {
  	.regs = sensor_contrast_pos1_regs,
  	.size = ARRAY_SIZE(sensor_contrast_pos1_regs),
  },
  {
  	.regs = sensor_contrast_pos2_regs,
  	.size = ARRAY_SIZE(sensor_contrast_pos2_regs),
  },
  {
  	.regs = sensor_contrast_pos3_regs,
  	.size = ARRAY_SIZE(sensor_contrast_pos3_regs),
  },
  {
  	.regs = sensor_contrast_pos4_regs,
  	.size = ARRAY_SIZE(sensor_contrast_pos4_regs),
  },
};

/*
 * The saturation setttings
 */
static struct regval_list sensor_saturation_neg4_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_neg1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_zero_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos1_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos2_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos3_regs[] = {
//NULL
};

static struct regval_list sensor_saturation_pos4_regs[] = {
//NULL
};

static struct cfg_array sensor_saturation[] = {
  {
  	.regs = sensor_saturation_neg4_regs,
  	.size = ARRAY_SIZE(sensor_saturation_neg4_regs),
  },
  {
  	.regs = sensor_saturation_neg3_regs,
  	.size = ARRAY_SIZE(sensor_saturation_neg3_regs),
  },
  {
  	.regs = sensor_saturation_neg2_regs,
  	.size = ARRAY_SIZE(sensor_saturation_neg2_regs),
  },
  {
  	.regs = sensor_saturation_neg1_regs,
  	.size = ARRAY_SIZE(sensor_saturation_neg1_regs),
  },
  {
  	.regs = sensor_saturation_zero_regs,
  	.size = ARRAY_SIZE(sensor_saturation_zero_regs),
  },
  {
  	.regs = sensor_saturation_pos1_regs,
  	.size = ARRAY_SIZE(sensor_saturation_pos1_regs),
  },
  {
  	.regs = sensor_saturation_pos2_regs,
  	.size = ARRAY_SIZE(sensor_saturation_pos2_regs),
  },
  {
  	.regs = sensor_saturation_pos3_regs,
  	.size = ARRAY_SIZE(sensor_saturation_pos3_regs),
  },
  {
  	.regs = sensor_saturation_pos4_regs,
  	.size = ARRAY_SIZE(sensor_saturation_pos4_regs),
  },
};

/*
 * The exposure target setttings
 */
static struct regval_list sensor_ev_neg4_regs[] = {
{0x038E,0x20},
{0x0381,0x32},
{0x0382,0x0E},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_neg3_regs[] = {
{0x038E,0x28},
{0x0381,0x3A},
{0x0382,0x16},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_neg2_regs[] = {
{0x038E,0x30},
{0x0381,0x42},
{0x0382,0x2E},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_neg1_regs[] = {
{0x038E,0x38},
{0x0381,0x5A},
{0x0382,0x26},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_zero_regs[] = {
{0x038E,0x40},
{0x0381,0x52},
{0x0382,0x2E},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_pos1_regs[] = {
{0x038E,0x48},
{0x0381,0x5A},
{0x0382,0x36},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_pos2_regs[] = {
{0x038E,0x50},
{0x0381,0x62},
{0x0382,0x3E},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_pos3_regs[] = {
{0x038E,0x58},
{0x0381,0x6A},
{0x0382,0x46},
{0x0000,0x01},
{0x0100,0x01},
};

static struct regval_list sensor_ev_pos4_regs[] = {
{0x038E,0x60},
{0x0381,0x72},
{0x0382,0x4E},
{0x0000,0x01},
{0x0100,0x01},
};

static struct cfg_array sensor_ev[] = {
  {
  	.regs = sensor_ev_neg4_regs,
  	.size = ARRAY_SIZE(sensor_ev_neg4_regs),
  },
  {
  	.regs = sensor_ev_neg3_regs,
  	.size = ARRAY_SIZE(sensor_ev_neg3_regs),
  },
  {
  	.regs = sensor_ev_neg2_regs,
  	.size = ARRAY_SIZE(sensor_ev_neg2_regs),
  },
  {
  	.regs = sensor_ev_neg1_regs,
  	.size = ARRAY_SIZE(sensor_ev_neg1_regs),
  },
  {
  	.regs = sensor_ev_zero_regs,
  	.size = ARRAY_SIZE(sensor_ev_zero_regs),
  },
  {
  	.regs = sensor_ev_pos1_regs,
  	.size = ARRAY_SIZE(sensor_ev_pos1_regs),
  },
  {
  	.regs = sensor_ev_pos2_regs,
  	.size = ARRAY_SIZE(sensor_ev_pos2_regs),
  },
  {
  	.regs = sensor_ev_pos3_regs,
  	.size = ARRAY_SIZE(sensor_ev_pos3_regs),
  },
  {
  	.regs = sensor_ev_pos4_regs,
  	.size = ARRAY_SIZE(sensor_ev_pos4_regs),
  },
};

/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 */


static struct regval_list sensor_fmt_yuv422_yuyv[] = {
	//YCbYCr
{0x0027,0x30}, // YCbYCr
{0x0000,0x01}, // CMU
};

static struct regval_list sensor_fmt_yuv422_yvyu[] = {
	//YCrYCb
{0x0027,0x38}, // YCrYCb
{0x0000,0x01}, // CMU
};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {
	//CrYCbY
{0x0027,0x18}, // CrYCbY
{0x0000,0x01}, // CMU
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {
	//CbYCrY
{0x0027,0x10}, // CbYCrY
{0x0000,0x01}, // CMU
};

static struct regval_list sensor_fmt_raw[] = {
	//raw
{0x0027,0x20}, // RAW
{0x0000,0x01}, // CMU
};



/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned short reg,
    unsigned char *value) //!!!!be careful of the para type!!!
{
	int ret=0;
	int cnt=0;

//  struct i2c_client *client = v4l2_get_subdevdata(sd);
  ret = cci_read_a16_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_read_a16_d8(sd,reg,value);
  	cnt++;
  }
  if(cnt>0)
  	vfe_dev_dbg("sensor read retry=%d\n",cnt);

  return ret;
}

static int sensor_write(struct v4l2_subdev *sd, unsigned short reg,
    unsigned char value)
{
	int ret=0;
	int cnt=0;

//  struct i2c_client *client = v4l2_get_subdevdata(sd);

  ret = cci_write_a16_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_write_a16_d8(sd,reg,value);
  	cnt++;
  }
  if(cnt>0)
  	vfe_dev_dbg("sensor write retry=%d\n",cnt);

  return ret;
}

/*
 * Write a list of register settings;
 */
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *regs, int array_size)
{
	printk("hm0357 sensor_write_array !\n");
	int i=0;

  if(!regs)
  	return 0;
  	//return -EINVAL;

  while(i<array_size)
  {
    if(regs->addr == REG_DLY) {
      msleep(regs->data);
    }
    else {
    	printk("write 0x%x=0x%x\n", regs->addr, regs->data);
      LOG_ERR_RET(sensor_write(sd, regs->addr, regs->data))
    }
    i++;
    regs++;
  }
  return 0;
}

static int sensor_g_hflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	//ret = sensor_write(sd, 0x0006, 0x02); // Mirror
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_hflip!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x0006, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	val &= (1<<0);
	val = val>>0;		//0x14 bit0 is mirror

	*value = val;

	info->hflip = *value;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	//ret = sensor_write(sd, 0x0006, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x14, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
		  val &= 0xfc;
			break;
		case 1:
			val |= (0x01|(info->vflip<<1));
			break;
		default:
			return -EINVAL;
	}
	//ret = sensor_write(sd, 0x14, val);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	
	usleep_range(10000,12000);
	
	info->hflip = value;
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	//ret = sensor_write(sd, 0x0006, 0x01);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_vflip!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x14, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_vflip!\n");
		return ret;
	}

	val &= (1<<1);
	val = val>>1;		//0x14 bit1 is upsidedown

	*value = val;

	info->vflip = *value;
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	//ret = sensor_write(sd, 0x0006, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x14, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_vflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
		  val &= 0xfc;
			break;
		case 1:
			val |= (0x02|info->hflip);
			break;
		default:
			return -EINVAL;
	}
	//ret = sensor_write(sd, 0x14, val);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}
	
	usleep_range(10000,12000);
	
	info->vflip = value;
	return 0;
}

static int sensor_g_autogain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_autogain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_autoexp(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	//ret = sensor_write(sd, 0x0380, 0xFF);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_autoexp!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x0380, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_autoexp!\n");
		return ret;
	}

	val &= 0x80;
	if (val == 0x80) {
		*value = V4L2_EXPOSURE_AUTO;
	}
	else
	{
		*value = V4L2_EXPOSURE_MANUAL;
	}

	info->autoexp = *value;
	return 0;
}

static int sensor_s_autoexp(struct v4l2_subdev *sd,
		enum v4l2_exposure_auto_type value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	//ret = sensor_write(sd, 0x0380, 0xFE);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x0380, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_autoexp!\n");
		return ret;
	}

	switch (value) {
		case V4L2_EXPOSURE_AUTO:
		  val |= 0xFF;
			break;
		case V4L2_EXPOSURE_MANUAL:
			val &= 0xFE;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			return -EINVAL;
		case V4L2_EXPOSURE_APERTURE_PRIORITY:
			return -EINVAL;
		default:
			return -EINVAL;
	}

	//ret = sensor_write(sd, 0x0380, val);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}
	
	usleep_range(10000,12000);
	
	info->autoexp = value;
	return 0;
}

static int sensor_g_autowb(struct v4l2_subdev *sd, int *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	//ret = sensor_write(sd, 0x0380, 0xFF);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_autowb!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x0380, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_autowb!\n");
		return ret;
	}

	val &= (1<<1);
	val = val>>1;		//0x22 bit1 is awb enable

	*value = val;
	info->autowb = *value;

	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
	unsigned char val;

	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		vfe_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}
	
	//ret = sensor_write(sd, 0x0380, 0xFD);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}
	//ret = sensor_read(sd, 0x0380, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_autowb!\n");
		return ret;
	}

	switch(value) {
	case 0:
		val &= 0xFF;
		break;
	case 1:
		val |= 0xFD;
		break;
	default:
		break;
	}	
	//ret = sensor_write(sd, 0x0380, val);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}
	
	usleep_range(10000,12000);
	
	info->autowb = value;
	return 0;
}

static int sensor_g_hue(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_hue(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}

static int sensor_g_gain(struct v4l2_subdev *sd, __s32 *value)
{
	return -EINVAL;
}

static int sensor_s_gain(struct v4l2_subdev *sd, int value)
{
	return -EINVAL;
}
/* *********************************************end of ******************************************** */

static int sensor_g_brightness(struct v4l2_subdev *sd, __s32 *value)
{
  struct sensor_info *info = to_state(sd);

  *value = info->brightness;
  return 0;
}

static int sensor_s_brightness(struct v4l2_subdev *sd, int value)
{
  struct sensor_info *info = to_state(sd);

  if(info->brightness == value)
    return 0;

  if(value < -4 || value > 4)
    return -ERANGE;

  LOG_ERR_RET(sensor_write_array(sd, sensor_brightness[value+4].regs, sensor_brightness[value+4].size))

  info->brightness = value;
  return 0;
}

static int sensor_g_contrast(struct v4l2_subdev *sd, __s32 *value)
{
  struct sensor_info *info = to_state(sd);

  *value = info->contrast;
  return 0;
}

static int sensor_s_contrast(struct v4l2_subdev *sd, int value)
{
  struct sensor_info *info = to_state(sd);

  if(info->contrast == value)
    return 0;

  if(value < -4 || value > 4)
    return -ERANGE;

  LOG_ERR_RET(sensor_write_array(sd, sensor_contrast[value+4].regs, sensor_contrast[value+4].size))

  info->contrast = value;
  return 0;
}

static int sensor_g_saturation(struct v4l2_subdev *sd, __s32 *value)
{
  struct sensor_info *info = to_state(sd);

  *value = info->saturation;
  return 0;
}

static int sensor_s_saturation(struct v4l2_subdev *sd, int value)
{
  struct sensor_info *info = to_state(sd);

  if(info->saturation == value)
    return 0;

  if(value < -4 || value > 4)
    return -ERANGE;

  LOG_ERR_RET(sensor_write_array(sd, sensor_saturation[value+4].regs, sensor_saturation[value+4].size))

  info->saturation = value;
  return 0;
}

static int sensor_g_exp_bias(struct v4l2_subdev *sd, __s32 *value)
{
  struct sensor_info *info = to_state(sd);

  *value = info->exp_bias;
  return 0;
}

static int sensor_s_exp_bias(struct v4l2_subdev *sd, int value)
{
  struct sensor_info *info = to_state(sd);

  if(info->exp_bias == value)
    return 0;

  if(value < -4 || value > 4)
    return -ERANGE;

  LOG_ERR_RET(sensor_write_array(sd, sensor_ev[value+4].regs, sensor_ev[value+4].size))

  info->exp_bias = value;
  return 0;
}

static int sensor_g_wb(struct v4l2_subdev *sd, int *value)
{
  struct sensor_info *info = to_state(sd);
  enum v4l2_auto_n_preset_white_balance *wb_type = (enum v4l2_auto_n_preset_white_balance*)value;

  *wb_type = info->wb;

  return 0;
}

static int sensor_s_wb(struct v4l2_subdev *sd,
    enum v4l2_auto_n_preset_white_balance value)
{
  struct sensor_info *info = to_state(sd);

  if(info->capture_mode == V4L2_MODE_IMAGE)
    return 0;
  
 // if(info->wb == value)
 //   return 0;
  
  LOG_ERR_RET(sensor_write_array(sd, sensor_wb[value].regs ,sensor_wb[value].size) )

  if (value == V4L2_WHITE_BALANCE_AUTO)
    info->autowb = 1;
  else
    info->autowb = 0;

	info->wb = value;
	return 0;
}

static int sensor_g_colorfx(struct v4l2_subdev *sd,
		__s32 *value)
{
	struct sensor_info *info = to_state(sd);
	enum v4l2_colorfx *clrfx_type = (enum v4l2_colorfx*)value;

	*clrfx_type = info->clrfx;
	return 0;
}

static int sensor_s_colorfx(struct v4l2_subdev *sd,
    enum v4l2_colorfx value)
{
  struct sensor_info *info = to_state(sd);

  if(info->clrfx == value)
    return 0;

  LOG_ERR_RET(sensor_write_array(sd, sensor_colorfx[value].regs, sensor_colorfx[value].size))

  info->clrfx = value;
  return 0;
}

static int sensor_g_flash_mode(struct v4l2_subdev *sd,
    __s32 *value)
{
  struct sensor_info *info = to_state(sd);
  enum v4l2_flash_led_mode *flash_mode = (enum v4l2_flash_led_mode*)value;

  *flash_mode = info->flash_mode;
  return 0;
}

static int sensor_s_flash_mode(struct v4l2_subdev *sd,
    enum v4l2_flash_led_mode value)
{
  struct sensor_info *info = to_state(sd);
//  struct vfe_dev *dev=(struct vfe_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
//  int flash_on,flash_off;
//
//  flash_on = (dev->flash_pol!=0)?1:0;
//  flash_off = (flash_on==1)?0:1;
//
//  switch (value) {
//  case V4L2_FLASH_MODE_OFF:
//    os_gpio_write(&dev->flash_io,flash_off);
//    break;
//  case V4L2_FLASH_MODE_AUTO:
//    return -EINVAL;
//    break;
//  case V4L2_FLASH_MODE_ON:
//    os_gpio_write(&dev->flash_io,flash_on);
//    break;
//  case V4L2_FLASH_MODE_TORCH:
//    return -EINVAL;
//    break;
//  case V4L2_FLASH_MODE_RED_EYE:
//    return -EINVAL;
//    break;
//  default:
//    return -EINVAL;
//  }

  info->flash_mode = value;
  return 0;
}

//static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
//{
//	int ret=0;
////	unsigned char rdval;
////
////	ret=sensor_read(sd, 0x00, &rdval);
////	if(ret!=0)
////		return ret;
////
////	if(on_off==CSI_STBY_ON)//sw stby on
////	{
////		ret=sensor_write(sd, 0x00, rdval&0x7f);
////	}
////	else//sw stby off
////	{
////		ret=sensor_write(sd, 0x00, rdval|0x80);
////	}
//	return ret;
//}

/*
 * Stuff that knows about the sensor.
 */

static int sensor_power(struct v4l2_subdev *sd, int on)
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);

  //make sure that no device can access i2c bus during sensor initial or power down
  //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
 // i2c_lock_adapter(client->adapter);

  //insure that clk_disable() and clk_enable() are called in pair
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF
  
  cci_lock(sd);
  
  switch(on)
  {
    case CSI_SUBDEV_STBY_ON:
      vfe_dev_dbg("CSI_SUBDEV_STBY_ON\n");
      //standby on io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      usleep_range(30000,31000);
      //inactive mclk after stadby in
      vfe_set_mclk(sd,OFF);
      break;
    case CSI_SUBDEV_STBY_OFF:
      vfe_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
      //active mclk before stadby out
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      usleep_range(30000,31000);
      //standby off io
      vfe_gpio_write(sd,PWDN,CSI_STBY_OFF);
      usleep_range(10000,12000);
    	//reset off io
	  vfe_gpio_write(sd,RESET,CSI_RST_OFF);
	  usleep_range(30000,31000);
      break;
    case CSI_SUBDEV_PWR_ON:
      vfe_dev_dbg("CSI_SUBDEV_PWR_ON\n");
      //power on reset
      vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
      vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
      usleep_range(10000,12000);
      //standby off io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
			//reset on io
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      //power supply
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_ON);
      vfe_set_pmu_channel(sd,IOVDD,ON);
      vfe_set_pmu_channel(sd,AVDD,ON);
      vfe_set_pmu_channel(sd,DVDD,ON);
      vfe_set_pmu_channel(sd,AFVDD,ON);
      usleep_range(20000,22000);
      //standby off io
      vfe_gpio_write(sd,PWDN,CSI_STBY_OFF);
      usleep_range(10000,12000);
			//active mclk
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      usleep_range(10000,12000);
			//reset on io
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
			usleep_range(30000,31000);
			//reset off io
      vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			usleep_range(30000,31000);
			break;
		case CSI_SUBDEV_PWR_OFF:
      vfe_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
      //reset io
      usleep_range(10000,12000);
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
			usleep_range(10000,12000);
			//inactive mclk after power off
      vfe_set_mclk(sd,OFF);
      //power supply off
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_OFF);
      vfe_set_pmu_channel(sd,AFVDD,OFF);
      vfe_set_pmu_channel(sd,DVDD,OFF);
      vfe_set_pmu_channel(sd,AVDD,OFF);
      vfe_set_pmu_channel(sd,IOVDD,OFF);  
      usleep_range(10000,12000);
      //standby and reset io
			//standby of io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      usleep_range(10000,12000);
      //set the io to hi-z
      vfe_gpio_set_status(sd,RESET,0);//set the gpio to input
      vfe_gpio_set_status(sd,PWDN,0);//set the gpio to input
			break;
		default:
			return -EINVAL;
	}

	//remember to unlock i2c adapter, so the device can access the i2c bus again
//	i2c_unlock_adapter(client->adapter);
	cci_unlock(sd);
	return 0;
}

static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
  switch(val)
  {
    case 0:
      vfe_gpio_write(sd,RESET,CSI_RST_OFF);
      usleep_range(10000,12000);
      break;
    case 1:
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      usleep_range(10000,12000);
      break;
    default:
      return -EINVAL;
  }

  return 0;
}

static int sensor_detect(struct v4l2_subdev *sd)
{
	int ret;
	unsigned char val;

	//ret = sensor_write(sd, 0xfe, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_detect!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x0001, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}
       /*******HM0357 ID = read(0x0001,0x0002=0x0357)******/
  pr_info("hm0357 sensor_detect id:0x%x\n", val);     
	if(val != 0x03)
		return -ENODEV;
	
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	vfe_dev_dbg("sensor_init\n");

	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		vfe_dev_err("chip found is not an target chip.\n");
		return ret;
	}
	return sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
}

static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	int ret=0;
		return ret;
}


/*
 * Store information about the video data format.
 */
static struct sensor_format_struct {
	__u8 *desc;
	//__u32 pixelformat;
	enum v4l2_mbus_pixelcode mbus_code;//linux-3.0
	struct regval_list *regs;
	int	regs_size;
	int bpp;   /* Bytes per pixel */
} sensor_formats[] = {
	{
		.desc		= "YUYV 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yuyv,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
		.bpp		= 2,
	},
	{
		.desc		= "YVYU 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_yvyu,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yvyu),
		.bpp		= 2,
	},
	{
		.desc		= "UYVY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_uyvy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_uyvy),
		.bpp		= 2,
	},
	{
		.desc		= "VYUY 4:2:2",
		.mbus_code	= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.regs 		= sensor_fmt_yuv422_vyuy,
		.regs_size = ARRAY_SIZE(sensor_fmt_yuv422_vyuy),
		.bpp		= 2,
	},
//	{
//		.desc		= "Raw RGB Bayer",
//		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
//		.regs 		= sensor_fmt_raw,
//		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
//		.bpp		= 1
//	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)


/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size
sensor_win_sizes[] = {
  /* VGA */
  {
    .width      = VGA_WIDTH,
    .height     = VGA_HEIGHT,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = NULL,
    .regs_size  = 0,
    .set_size   = NULL,
  },
};

#define N_WIN_SIZES (ARRAY_SIZE(sensor_win_sizes))

static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                 enum v4l2_mbus_pixelcode *code)
{
  if (index >= N_FMTS)
    return -EINVAL;

  *code = sensor_formats[index].mbus_code;
  return 0;
}

static int sensor_enum_size(struct v4l2_subdev *sd,
                            struct v4l2_frmsizeenum *fsize)
{
  if(fsize->index > N_WIN_SIZES-1)
  	return -EINVAL;

  fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
  fsize->discrete.width = sensor_win_sizes[fsize->index].width;
  fsize->discrete.height = sensor_win_sizes[fsize->index].height;

  return 0;
}


static int sensor_try_fmt_internal(struct v4l2_subdev *sd,
    struct v4l2_mbus_framefmt *fmt,
    struct sensor_format_struct **ret_fmt,
    struct sensor_win_size **ret_wsize)
{
  int index;
  struct sensor_win_size *wsize;

  for (index = 0; index < N_FMTS; index++)
    if (sensor_formats[index].mbus_code == fmt->code)
      break;

  if (index >= N_FMTS)
    return -EINVAL;

  if (ret_fmt != NULL)
    *ret_fmt = sensor_formats + index;

  /*
   * Fields: the sensor devices claim to be progressive.
   */

  fmt->field = V4L2_FIELD_NONE;

  /*
   * Round requested image size down to the nearest
   * we support, but not below the smallest.
   */
  for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
       wsize++)
    if (fmt->width >= wsize->width && fmt->height >= wsize->height)
      break;

  if (wsize >= sensor_win_sizes + N_WIN_SIZES)
    wsize--;   /* Take the smallest one */
  if (ret_wsize != NULL)
    *ret_wsize = wsize;
  /*
   * Note the size we'll actually handle.
   */
  fmt->width = wsize->width;
  fmt->height = wsize->height;
  //pix->bytesperline = pix->width*sensor_formats[index].bpp;
  //pix->sizeimage = pix->height*pix->bytesperline;

  return 0;
}

static int sensor_try_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}

static int sensor_g_mbus_config(struct v4l2_subdev *sd,
           struct v4l2_mbus_config *cfg)
{
  cfg->type = V4L2_MBUS_PARALLEL;
  cfg->flags = V4L2_MBUS_MASTER | VREF_POL | HREF_POL | CLK_POL ;

  return 0;
}

/*
 * Set a format.
 */
static int sensor_s_fmt(struct v4l2_subdev *sd,
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);
	vfe_dev_dbg("sensor_s_fmt\n");
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret)
		return ret;


	sensor_write_array(sd, sensor_fmt->regs , sensor_fmt->regs_size);

	ret = 0;
	if (wsize->regs)
	{
		ret = sensor_write_array(sd, wsize->regs , wsize->regs_size);
		if (ret < 0)
			return ret;
	}

	if (wsize->set_size)
	{
		ret = wsize->set_size(sd);
		if (ret < 0)
			return ret;
	}

	info->fmt = sensor_fmt;
	info->width = wsize->width;
	info->height = wsize->height;

	return 0;
}

/*
 * Implement G/S_PARM.  There is a "high quality" mode we could try
 * to do someday; for now, we just do the frame rate tweak.
 */
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = SENSOR_FRAME_RATE;

	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
//	struct v4l2_captureparm *cp = &parms->parm.capture;
	//struct v4l2_fract *tpf = &cp->timeperframe;
	//struct sensor_info *info = to_state(sd);
	//int div;

//	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
//		return -EINVAL;
//	if (cp->extendedmode != 0)
//		return -EINVAL;

//	if (tpf->numerator == 0 || tpf->denominator == 0)
//		div = 1;  /* Reset to full rate */
//	else
//		div = (tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator;
//
//	if (div == 0)
//		div = 1;
//	else if (div > CLK_SCALE)
//		div = CLK_SCALE;
//	info->clkrc = (info->clkrc & 0x80) | div;
//	tpf->numerator = 1;
//	tpf->denominator = sensor_FRAME_RATE/div;
//sensor_write(sd, REG_CLKRC, info->clkrc);
	return 0;
}


/*
 * Code for dealing with controls.
 * fill with different sensor module
 * different sensor module has different settings here
 * if not support the follow function ,retrun -EINVAL
 */

/* *********************************************begin of ******************************************** */
static int sensor_queryctrl(struct v4l2_subdev *sd,
		struct v4l2_queryctrl *qc)
{
	/* Fill in min, max, step and default value for these controls. */
	/* see include/linux/videodev2.h for details */
	/* see sensor_s_parm and sensor_g_parm for the meaning of value */

	switch (qc->id) {
//	case V4L2_CID_BRIGHTNESS:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_CONTRAST:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_SATURATION:
//		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 1);
//	case V4L2_CID_HUE:
//		return v4l2_ctrl_query_fill(qc, -180, 180, 5, 0);
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//	case V4L2_CID_GAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 255, 1, 128);
//	case V4L2_CID_AUTOGAIN:
//		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_EXPOSURE:
  case V4L2_CID_AUTO_EXPOSURE_BIAS:
		return v4l2_ctrl_query_fill(qc, -4, 4, 1, 0);
	case V4L2_CID_EXPOSURE_AUTO:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
  case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
    return v4l2_ctrl_query_fill(qc, 0, 9, 1, 1);
	case V4L2_CID_AUTO_WHITE_BALANCE:
		return v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
	case V4L2_CID_COLORFX:
    return v4l2_ctrl_query_fill(qc, 0, 15, 1, 0);
  case V4L2_CID_FLASH_LED_MODE:
	  return v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);
	}
	return -EINVAL;
}


static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_g_brightness(sd, &ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_g_contrast(sd, &ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_g_saturation(sd, &ctrl->value);
	case V4L2_CID_HUE:
		return sensor_g_hue(sd, &ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_g_vflip(sd, &ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_g_hflip(sd, &ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_g_gain(sd, &ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_g_autogain(sd, &ctrl->value);
	case V4L2_CID_EXPOSURE:
  case V4L2_CID_AUTO_EXPOSURE_BIAS:
    return sensor_g_exp_bias(sd, &ctrl->value);
  case V4L2_CID_EXPOSURE_AUTO:
    return sensor_g_autoexp(sd, &ctrl->value);
  case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
    return sensor_g_wb(sd, &ctrl->value);
  case V4L2_CID_AUTO_WHITE_BALANCE:
    return sensor_g_autowb(sd, &ctrl->value);
  case V4L2_CID_COLORFX:
    return sensor_g_colorfx(sd, &ctrl->value);
  case V4L2_CID_FLASH_LED_MODE:
    return sensor_g_flash_mode(sd, &ctrl->value);
	}
	return -EINVAL;
}

static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
  struct v4l2_queryctrl qc;
  int ret;

//  vfe_dev_dbg("sensor_s_ctrl ctrl->id=0x%8x\n", ctrl->id);
  qc.id = ctrl->id;
  ret = sensor_queryctrl(sd, &qc);
  if (ret < 0) {
    return ret;
  }

	if (qc.type == V4L2_CTRL_TYPE_MENU ||
		qc.type == V4L2_CTRL_TYPE_INTEGER ||
		qc.type == V4L2_CTRL_TYPE_BOOLEAN)
	{
	  if (ctrl->value < qc.minimum || ctrl->value > qc.maximum) {
	    return -ERANGE;
	  }
	}
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		return sensor_s_brightness(sd, ctrl->value);
	case V4L2_CID_CONTRAST:
		return sensor_s_contrast(sd, ctrl->value);
	case V4L2_CID_SATURATION:
		return sensor_s_saturation(sd, ctrl->value);
	case V4L2_CID_HUE:
		return sensor_s_hue(sd, ctrl->value);
	case V4L2_CID_VFLIP:
		return sensor_s_vflip(sd, ctrl->value);
	case V4L2_CID_HFLIP:
		return sensor_s_hflip(sd, ctrl->value);
	case V4L2_CID_GAIN:
		return sensor_s_gain(sd, ctrl->value);
	case V4L2_CID_AUTOGAIN:
		return sensor_s_autogain(sd, ctrl->value);
	case V4L2_CID_EXPOSURE:
    case V4L2_CID_AUTO_EXPOSURE_BIAS:
      return sensor_s_exp_bias(sd, ctrl->value);
    case V4L2_CID_EXPOSURE_AUTO:
      return sensor_s_autoexp(sd,
          (enum v4l2_exposure_auto_type) ctrl->value);
    case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
  		return sensor_s_wb(sd,
          (enum v4l2_auto_n_preset_white_balance) ctrl->value);
    case V4L2_CID_AUTO_WHITE_BALANCE:
      return sensor_s_autowb(sd, ctrl->value);
    case V4L2_CID_COLORFX:
      return sensor_s_colorfx(sd,
          (enum v4l2_colorfx) ctrl->value);
    case V4L2_CID_FLASH_LED_MODE:
      return sensor_s_flash_mode(sd,
          (enum v4l2_flash_led_mode) ctrl->value);
	}
	return -EINVAL;
}


static int sensor_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}


/* ----------------------------------------------------------------------- */

static const struct v4l2_subdev_core_ops sensor_core_ops = {
	.g_chip_ident = sensor_g_chip_ident,
	.g_ctrl = sensor_g_ctrl,
	.s_ctrl = sensor_s_ctrl,
	.queryctrl = sensor_queryctrl,
	.reset = sensor_reset,
	.init = sensor_init,
	.s_power = sensor_power,
	.ioctl = sensor_ioctl,
};

static const struct v4l2_subdev_video_ops sensor_video_ops = {
  .enum_mbus_fmt = sensor_enum_fmt,
  .enum_framesizes = sensor_enum_size,
  .try_mbus_fmt = sensor_try_fmt,
  .s_mbus_fmt = sensor_s_fmt,
  .s_parm = sensor_s_parm,
  .g_parm = sensor_g_parm,
  .g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = SENSOR_NAME,
	.addr_width = CCI_BITS_16,
	.data_width = CCI_BITS_8,
};


static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct sensor_info *info;
//	int ret;

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
	if (info == NULL)
		return -ENOMEM;
	sd = &info->sd;
//	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

cci_dev_probe_helper(sd, client, &sensor_ops, &cci_drv);

	info->fmt = &sensor_formats[0];
	info->brightness = 0;
	info->contrast = 0;
	info->saturation = 0;
	info->hue = 0;
	info->hflip = 0;
	info->vflip = 0;
	info->gain = 0;
	info->autogain = 1;
	info->exp = 0;
	info->autoexp = 0;
	info->autowb = 1;
	info->wb = 0;
	info->clrfx = 0;

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
//	struct v4l2_subdev *sd = i2c_get_clientdata(client);

//	v4l2_device_unregister_subdev(sd);

	struct v4l2_subdev *sd;

	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
	.name = SENSOR_NAME,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

static __init int init_sensor(void)
{
//	return i2c_add_driver(&sensor_driver);
return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
 // i2c_del_driver(&sensor_driver);
 cci_dev_exit_helper(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
