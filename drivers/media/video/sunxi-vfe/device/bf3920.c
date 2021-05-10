/*
 * A V4L2 driver for GalaxyCore bf3920 cameras.
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
MODULE_DESCRIPTION("A low-level driver for GalaxyCore bf3920 sensors");
MODULE_LICENSE("GPL");



//for internel driver debug
#define DEV_DBG_EN   		0
#if(DEV_DBG_EN == 1)
#define vfe_dev_dbg(x,arg...) printk("[CSI_DEBUG][BF3920]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...)
#endif

#define vfe_dev_err(x,arg...) printk("[CSI_ERR][BF3920]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[CSI][BF3920]"x,##arg)

#define LOG_ERR_RET(x)  { \
                          int ret;  \
                          ret = x; \
                          if(ret < 0) {\
                            vfe_dev_err("error at %s\n",__func__);  \
                            return ret; \
                          } \
                        }

//define module timing
#define MCLK              (24*1000*1000)
#define VREF_POL          V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL          V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL           V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR 0x3920

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0

#define regval_list reg_list_a8_d8
#define REG_TERM 0xff
#define VAL_TERM 0xff
#define REG_DLY  0xff

/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 8



/*
 * The bf3920 sits on i2c with ID 0xDD
 */
#define I2C_ADDR                    0xDE

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



/*
 * The default register settings
 *
 */

static struct regval_list sensor_default_regs[] = {

{0x1b,0x2c},
{0x11,0x30},
{0x8a,0x78},
{0x2a,0x0c},
{0x2b,0x90},
{0x92,0x02},
{0x93,0x00},

{0x09,0x42},
{0x21,0x15},
{0x15,0x82},
{0x3a,0x00},

//initial AE and AWB
{0x13,0x00},
//{0x01,0x12},
//{0x02,0x22},
{0x24,0xc8},
{0x87,0x2d},
{0x13,0x07},

//black level
{0x27,0x98},
{0x28,0xff},
{0x29,0x60},

//black target
{0x1F,0x20},
{0x22,0x20},
{0x20,0x20},
{0x26,0x20},

//analog
{0xe2,0xc4},
{0xe7,0x4e},
{0x08,0x16},
{0x16,0x28},
{0x1c,0x00},
{0x1d,0xf1},
{0xbb,0x30},
{0xf9,0x00},
{0xed,0x8f},
{0x3e,0x80},

//Shading
{0x1e,0x46},
{0x35,0x30},
{0x65,0x30},
{0x66,0x2a},
{0xbc,0xd3},
{0xbd,0x5c},
{0xbe,0x24},
{0xbf,0x13},
{0x9b,0x5c},
{0x9c,0x24},
{0x36,0x13},
{0x37,0x5c},
{0x38,0x24},

//denoise
{0x70,0x01},
{0x72,0xe2},//62 f2 //yang
{0x78,0x37},
{0x7a,0x36},//29 28 //yang
{0x7d,0x85},
#if 1 

//AE gain curve
{0x13,0x0f},
{0x8a,0x10},
{0x8b,0x1d},
{0x8e,0x1e},
{0x8f,0x3a},
{0x94,0x3d},
{0x95,0x71},
{0x96,0x76},
{0x97,0xc5},
{0x98,0xc6},
{0x99,0x7b}, 
 	
//AE
{0x13,0x07},
{0x24,0x45},// AE Target 40 ///yang
{0x25,0x88},// Bit[7:4]: AE_LOC_INT; Bit[3:0]:AE_LOC_GLB
{0x97,0x30},// Y target value1.
{0x98,0x08},// AE ���ں�Ȩ��  0x0a zzg
{0x80,0x92},//Bit[3:2]: the bigger, Y_AVER_MODIFY is smaller     0x92
{0x81,0xf0},//AE speed  //0xe0//ff e0 //yang
{0x82,0x22},//minimum global gain   0x30 zzg//22
{0x83,0x44}, //0x60 zzg//44
{0x84,0x45}, ///0x65 zzg//45
{0x85,0x5d}, ///0x90 zzg//5d
{0x86,0x90},//maximum gain zzg 0xc0 0x97 ///70 //yang
{0x89,0x95},//INT_MAX_MID    9d zzg 6d //yang
//{0x8a,0xbf},//50 hz banding
//{0x8b,0x9f},//60hz  banding
{0x94,0x62},//82 //yang Bit[7:4]: Threshold for over exposure pixels, the smaller, the more over exposure pixels; Bit[3:0]: Control the start of AE. 0x42 zzg//b2
#else
// AE
{0x13,0x07},
{0x24,0x48},
{0x25,0x88},
{0x97,0x30},
{0x98,0x0a},
{0x80,0x92},
{0x81,0xe0},
{0x82,0x30},
{0x83,0x60},
{0x84,0x65},
{0x85,0x90},
{0x86,0xc0},
{0x89,0x9d},
{0x94,0x42},
#endif
//Gamma default
{0x3f,0x28}, ///20 //yang
{0x39,0xa8}, ///a0 //yang

/*
{0x40,0x20},
{0x41,0x25},
{0x42,0x23},
{0x43,0x1f},
{0x44,0x18},
{0x45,0x15},
{0x46,0x12},
{0x47,0x10},
{0x48,0x10},
{0x49,0x0f},
{0x4b,0x0e},
{0x4c,0x0c},
{0x4e,0x0b},
{0x4f,0x09},//0a
{0x50,0x07},
*/


//gamma ���ع��Ⱥã�����
{0x40,0x28},
{0x41,0x28},
{0x42,0x30},
{0x43,0x29},
{0x44,0x23},
{0x45,0x1b},
{0x46,0x17},
{0x47,0x0f},
{0x48,0x0d},
{0x49,0x0b},
{0x4b,0x09},
{0x4c,0x08},
{0x4e,0x07},
{0x4f,0x05},
{0x50,0x04},

//{0x39,0xa0},
//{0x3f,0x20},

//gamma ��������
/*{0x40,0x28},
{0x41,0x26},
{0x42,0x24},
{0x43,0x22},
{0x44,0x1c},
{0x45,0x19},
{0x46,0x15},
{0x47,0x11},
{0x48,0x0f},
{0x49,0x0e},
{0x4b,0x0c},
{0x4c,0x0a},
{0x4e,0x09},
{0x4f,0x08},
{0x50,0x04},
{0x39,0xa0},
{0x3f,0x20},
*/


{0x5c,0x80},
{0x51,0x22},
{0x52,0x00},
{0x53,0x96},
{0x54,0x8C},
{0x57,0x7F},
{0x58,0x0B},
{0x5a,0x14},

//Color default
{0x5c,0x00},
{0x51,0x32},
{0x52,0x17},
{0x53,0x8C},
{0x54,0x79},
{0x57,0x6E},
{0x58,0x01},
{0x5a,0x36},
{0x5e,0x38},


//color ��ɫ��
{0x5c,0x00},
{0x51,0x24},
{0x52,0x0b},
{0x53,0x77},
{0x54,0x65},
{0x57,0x5e},
{0x58,0x19},
{0x5a,0x16},
{0x5e,0x38},

/*

//color ����
{0x5c,0x00},
{0x51,0x2b},
{0x52,0x0e},
{0x53,0x9f},
{0x54,0x7d},
{0x57,0x91},
{0x58,0x21},
{0x5a,0x16},
{0x5e,0x38},


//color ɫ�ʵ�
{0x5c,0x00},
{0x51,0x24},
{0x52,0x0b},
{0x53,0x77},
{0x54,0x65},
{0x57,0x5e},
{0x58,0x19},
{0x5a,0x16},
{0x5e,0x38},
*/

//AWB
{0x6a,0x81},
{0x23,0x66},
{0xa1,0x31},
{0xa2,0x0b},
{0xa3,0x25},
{0xa4,0x09},
{0xa5,0x26},
{0xa7,0x9a},
{0xa8,0x15},
{0xa9,0x13},
{0xaa,0x12},
{0xab,0x16},
{0xc8,0x10},
{0xc9,0x15},
{0xd2,0x78},
{0xd4,0x20},

//saturation
{0x56,0x28},
{0x55,0x00},//when 0x56[7:6]=2'b0x,0x55[7:0]:Brightness control.bit[7]: 0:positive  1:negative [6:0]:value
{0xb1,0x95},//when 0x56[7:6]=2'b0x,0xb1[7]:auto contrast switch(0:on 1:off)
{0xb0,0x8d},

{0x56,0xe8},
{0x55,0xf0},
{0xb0,0xb8},

{0x56,0xa8},
{0x55,0xc5},//
{0xb0,0x93},  ///98 //yang

//skin
{0xee,0x2a},
{0xef,0x1b},
};

/* 1600X1200 UXGA capture */
static struct regval_list sensor_uxga_regs[] ={

{0x1b,0x2c},
{0x11,0x30},
{0x8a,0x78},
{0x2a,0x0c},
{0x2b,0x90},
{0x92,0x02},
{0x93,0x00},


//window
{0x4a,0x80},
{0xcc,0x00},
{0xca,0x00},
{0xcb,0x00},
{0xcf,0x00},
{0xcd,0x00},
{0xce,0x00},
{0xc0,0x00},
{0xc1,0x00},
{0xc2,0x00},
{0xc3,0x00},
{0xc4,0x00},
{0xb5,0x00},
{0x03,0x60},
{0x17,0x00},
{0x18,0x40},
{0x10,0x40},
{0x19,0x00},
{0x1a,0xb0},
{0x0B,0x00},


};

/* 800X600 SVGA,30fps*/
static struct regval_list sensor_svga_regs[] =
{
{0x1b,0x2c},
{0x11,0x30},
{0x8a,0x78},
{0x2a,0x0c},
{0x2b,0x90},
{0x92,0x02},
{0x93,0x00},

#if 1

//window
{0x4a,0x80},
{0xcc,0x00},
{0xca,0x00},
{0xcb,0x00},
{0xcf,0x00},
{0xcd,0x00},
{0xce,0x00},
{0xc0,0x00},
{0xc1,0x00},
{0xc2,0x00},
{0xc3,0x00},
{0xc4,0x00},
{0xb5,0x00},
{0x03,0x60},
{0x17,0x00},
{0x18,0x40},
{0x10,0x40},
{0x19,0x00},
{0x1a,0xb0},
{0x0B,0x10}, 

#else
//1024*768
{0x4a,0x80},
{0xcc,0x00},
{0xca,0x00},
{0xcb,0x00},
{0xcf,0x00},
{0xcd,0x00},
{0xce,0x00},
{0xc0,0xc3},
{0xc1,0x04},
{0xc2,0x04},
{0xc3,0x11},
{0xc4,0x8f},
{0xb5,0x8f},
{0x03,0x40},
{0x17,0x00},
{0x18,0x00},
{0x10,0x30},
{0x19,0x00},
{0x1a,0x00},
{0x0B,0x00}, 

#endif


};

////1280*720---init---///
//static struct regval_list Gc2015_sensor_hd720_regs[] = {
//
//
//{0xfe , 0x00},
//{0x05, 0x01},
//{0x06, 0x9e},
//{0x07, 0x01},
//{0x08, 0x6d},
//{0x0a , 0xf0}, //row start
//{0x0c , 0xa0}, //col start
//{0x0d , 0x02},
//{0x0e , 0xd8},
//{0x0f , 0x05}, //Window setting
//{0x10 , 0x18},
//
//{0xfe, 0x01},
//{0x27, 0x00},
//{0x28, 0xd9},
//{0x29, 0x04},
//{0x2a, 0x3d},//18fps
//{0x2b, 0x06},
//{0x2c, 0xc8},//12.5fps
//{0x2d, 0x0a},
//{0x2e, 0x2c},//8fps
//{0x3e, 0x40},//0x40 0x00
//
////measure window
//{0xfe, 0x00},
//{0xec, 0x04},
//{0xed, 0x04},
//{0xee, 0x50},
//{0xef, 0x58},
//
//
//
//
//{0x90 , 0x01},  //crop enable
//{0x95 , 0x02},
//{0x96 , 0xd0},
//{0x97 , 0x05},
//{0x98 , 0x00},
//
//
//{0xfe , 0x03},
//{0x42 , 0x80},
//{0x43 , 0x06}, //output buf width
//{0x41 , 0x00}, // delay
//{0x40 , 0x00}, //fifo half full trig
//{0x17 , 0x01}, //widv
//{0xfe , 0x00},
//
//
//{0x99, 0x11},
//{0xc8, 0x00},
//
//{0xfa, 0x11},
//
//
//
//
//
//{0xff, 0xff},
//
//
//
//};



/*
 * The white balance settings
 * Here only tune the R G B channel gain.
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_manual[] = {
//null
};

static struct regval_list sensor_wb_auto_regs[] = {

	{0x13, 0x07},

};

static struct regval_list sensor_wb_incandescence_regs[] = {
	//bai re guang
        {0x13, 0x05},
	{0x01, 0x1d},
	{0x02, 0x16},
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//ri guang deng
        {0x13, 0x05},
	{0x01, 0x28},
	{0x02, 0x0a},
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng
        {0x13, 0x05},
	{0x01, 0x20},
	{0x02, 0x10},
};

static struct regval_list sensor_wb_horizon[] = {
//null
};
static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
	 //Sunny
        {0x13, 0x05},
	{0x01, 0x10},
	{0x02, 0x20},
};

static struct regval_list sensor_wb_flash[] = {
//null
};

static struct regval_list sensor_wb_cloud_regs[] = {
        {0x13, 0x05},
	{0x01, 0x0e},
	{0x02, 0x24},
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

	{0x98, 0x8a},
	{0x70, 0x01},
	{0x69, 0x00},
	{0x67, 0x80},
	{0x68, 0x80},
	{0x56, 0x28},
	{0xb0, 0x8d},

	{0x56, 0x28},
	{0xb1, 0x15},//0x95
	{0x56, 0xe8},
	{0xb4, 0x43},//default//0x40
};

static struct regval_list sensor_colorfx_bw_regs[] = {

};

static struct regval_list sensor_colorfx_sepia_regs[] = {
{0x98,0x8a},
{0x70,0x00},
{0x69,0x20},
{0x67,0x58},
{0x68,0xa0},
{0x56,0x28},
{0xb0,0x8d},
{0xb1,0x95},
{0x56,0xe8},
{0xb4,0x40},


};

static struct regval_list sensor_colorfx_negative_regs[] = {


};

static struct regval_list sensor_colorfx_emboss_regs[] = {
{0x98,0x8a},
{0x70,0x11},
{0x69,0x00},
{0x67,0x80},
{0x68,0x80},
{0x56,0x28},
{0xb0,0x8d},
{0xb1,0x95},
{0x56,0xe8},
{0xb4,0x40},

};

static struct regval_list sensor_colorfx_sketch_regs[] = {
//NULL
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {


};

static struct regval_list sensor_colorfx_grass_green_regs[] = {

};

static struct regval_list sensor_colorfx_skin_whiten_regs[] = {
//NULL
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

};

static struct regval_list sensor_contrast_neg3_regs[] = {

};

static struct regval_list sensor_contrast_neg2_regs[] = {

};

static struct regval_list sensor_contrast_neg1_regs[] = {

};

static struct regval_list sensor_contrast_zero_regs[] = {

};

static struct regval_list sensor_contrast_pos1_regs[] = {

};

static struct regval_list sensor_contrast_pos2_regs[] = {

};

static struct regval_list sensor_contrast_pos3_regs[] = {

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

};

static struct regval_list sensor_ev_neg3_regs[] = {
  {0x56, 0x28},
	{0x55, 0xc8},

};

static struct regval_list sensor_ev_neg2_regs[] = {
{0x56, 0x28},
	{0x55, 0xb0},

};

static struct regval_list sensor_ev_neg1_regs[] = {
{0x56, 0x28},
	{0x55, 0x98},

};

static struct regval_list sensor_ev_zero_regs[] = {
 {0x56, 0x28},
	{0x55, 0x00},
};

static struct regval_list sensor_ev_pos1_regs[] = {
 {0x56, 0x28},
	{0x55, 0x18},
};

static struct regval_list sensor_ev_pos2_regs[] = {
{0x56, 0x28},
	{0x55, 0x30},

};

static struct regval_list sensor_ev_pos3_regs[] = {
  {0x56, 0x28},
	{0x55, 0x48},
};

static struct regval_list sensor_ev_pos4_regs[] = {

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
		{0x3a,0x00},		//YCbYCr
};


static struct regval_list sensor_fmt_yuv422_yvyu[] = {
		{0x3a,0x01},		//YCrYCb
};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {
		{0x3a,0x03},	//CrYCbY
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {
		{0x3a,0x02},		//CbYCrY
};

static struct regval_list sensor_fmt_raw[] = {

};



/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned char reg,
    unsigned char *value) //!!!!be careful of the para type!!!
{
	int ret=0;
	int cnt=0;

  ret = cci_read_a8_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_read_a8_d8(sd,reg,value);
  	cnt++;
  }
  if(cnt>0)
  	vfe_dev_dbg("sensor read retry=%d\n",cnt);

  return ret;
}

static int sensor_write(struct v4l2_subdev *sd, unsigned char reg,
    unsigned char value)
{
	int ret=0;
	int cnt=0;


  ret = cci_write_a8_d8(sd,reg,value);
  while(ret!=0&&cnt<2)
  {
  	ret = cci_write_a8_d8(sd,reg,value);
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
	int i=0;

  if(!regs)
  	return -EINVAL;

  while(i<array_size)
  {
    if(regs->addr == REG_DLY) {
      msleep(regs->data);
    }
    else {
    	//printk("write 0x%x=0x%x\n", regs->addr, regs->data);
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
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write(sd, 0x1e, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_hflip!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x1e, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_hflip!\n");
		return ret;
	}

	val &= (1<<0);
	val = val>>0;		//0x29 bit0 is mirror

	*value = val;

	info->hflip = *value;
	return 0;
}

static int sensor_s_hflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write(sd, 0x1e, 20);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	ret = sensor_read(sd, 0x1e, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
		  val &= 0xdf;
			break;
		case 1:
			val |= 0x20;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, 0x1e, val);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}

	usleep_range(20000,22000);

	info->hflip = value;
	return 0;
}

static int sensor_g_vflip(struct v4l2_subdev *sd, __s32 *value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write(sd, 0x1e, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_vflip!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x1e, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_vflip!\n");
		return ret;
	}

	val &= (1<<1);
	val = val>>1;		//0x29 bit1 is upsidedown

	*value = val;

	info->vflip = *value;
	return 0;
}

static int sensor_s_vflip(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write(sd, 0x1e, 0x10);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x1e, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_vflip!\n");
		return ret;
	}

	switch (value) {
		case 0:
		  val &= 0xef;
			break;
		case 1:
			val |= 0x10;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, 0x1e, val);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}

	usleep_range(20000,22000);

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
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write(sd, 0x13, 0x07);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_autoexp!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x13, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_autoexp!\n");
		return ret;
	}

	val &= 0x01;
	if (val == 0x01) {
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
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write(sd, 0x13, 0x07);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x13, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_autoexp!\n");
		return ret;
	}

	switch (value) {
		case V4L2_EXPOSURE_AUTO:
		  val |= 0x01;
			break;
		case V4L2_EXPOSURE_MANUAL:
			val &= 0xfe;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			return -EINVAL;
		case V4L2_EXPOSURE_APERTURE_PRIORITY:
			return -EINVAL;
		default:
			return -EINVAL;
	}

	//ret = sensor_write(sd, 0xb6, val);
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
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write(sd, 0x13, 0x05);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_autowb!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x13, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_g_autowb!\n");
		return ret;
	}

	val &= (1<<1);
	val = val>>1;		//0x42 bit1 is awb enable

	*value = val;
	info->autowb = *value;

	return 0;
}

static int sensor_s_autowb(struct v4l2_subdev *sd, int value)
{
	int ret;
	struct sensor_info *info = to_state(sd);
//	struct regval_list regs;
	unsigned char val;
	return 0;
	ret = sensor_write_array(sd, sensor_wb_auto_regs, ARRAY_SIZE(sensor_wb_auto_regs));
	if (ret < 0) {
		vfe_dev_err("sensor_write_array err at sensor_s_autowb!\n");
		return ret;
	}

	ret = sensor_read(sd, 0x13, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_autowb!\n");
		return ret;
	}

	switch(value) {
	case 0:
		val &= 0xfa;
		break;
	case 1:
		val |= 0x05;
		break;
	default:
		break;
	}
	ret = sensor_write(sd, 0x13, val);
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


static struct regval_list sensor_oe_disable_regs[] = {
	//{0x3002,0x00},
  //{REG_TERM,VAL_TERM},
};

static struct regval_list sensor_oe_enable_regs[] = {
  //{0x3002,0xe4},
  //{REG_TERM,VAL_TERM},
};




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

  if(info->wb == value)
    return 0;

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

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret=0;
//	unsigned char rdval;
//
//	ret=sensor_read(sd, 0x00, &rdval);
//	if(ret!=0)
//		return ret;
//
//	if(on_off==CSI_STBY_ON)//sw stby on
//	{
//		ret=sensor_write(sd, 0x00, rdval&0x7f);
//	}
//	else//sw stby off
//	{
//		ret=sensor_write(sd, 0x00, rdval|0x80);
//	}
	return ret;
}

/*
 * Stuff that knows about the sensor.
 */

static int sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret;

  //make sure that no device can access i2c bus during sensor initial or power down
  //when using i2c_lock_adpater function, the following codes must not access i2c bus before calling i2c_unlock_adapter
  cci_lock(sd);

  //insure that clk_disable() and clk_enable() are called in pair
  //when calling CSI_SUBDEV_STBY_ON/OFF and CSI_SUBDEV_PWR_ON/OFF
  switch(on)
  {
    case CSI_SUBDEV_STBY_ON:
      vfe_dev_dbg("CSI_SUBDEV_STBY_ON\n");
      //disable io oe
      vfe_dev_print("disalbe oe!\n");
      ret = sensor_write_array(sd, sensor_oe_disable_regs, ARRAY_SIZE(sensor_oe_disable_regs));
      if(ret < 0)
        vfe_dev_err("disalbe oe falied!\n");
      //software standby on
      ret = sensor_s_sw_stby(sd, CSI_STBY_ON);
      if(ret < 0)
        vfe_dev_err("soft stby falied!\n");
      //standby on io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      usleep_range(10000,12000);
      //inactive mclk after stadby in
      vfe_set_mclk(sd,OFF);
      break;
    case CSI_SUBDEV_STBY_OFF:
      vfe_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
      //software standby
      ret = sensor_s_sw_stby(sd, CSI_STBY_OFF);
      if(ret < 0)
        vfe_dev_err("soft stby off falied!\n");
      usleep_range(10000,12000);
      //standby off io
      vfe_gpio_write(sd,PWDN,CSI_STBY_OFF);
      usleep_range(10000,12000);
      //active mclk before stadby out
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      usleep_range(10000,12000);
      vfe_dev_print("enable oe!\n");
      ret = sensor_write_array(sd, sensor_oe_enable_regs,  ARRAY_SIZE(sensor_oe_enable_regs));
      if(ret < 0)
        vfe_dev_err("enable oe falied!\n");
      break;
    case CSI_SUBDEV_PWR_ON:
      vfe_dev_dbg("CSI_SUBDEV_PWR_ON\n");
      //power on reset
      vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
      vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
      //power down io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      //reset on io
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      usleep_range(1000,1200);
      //active mclk before power on
      vfe_set_mclk_freq(sd,MCLK);
      vfe_set_mclk(sd,ON);
      usleep_range(10000,12000);
      //power supply
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_ON);
      vfe_set_pmu_channel(sd,IOVDD,ON);
      vfe_set_pmu_channel(sd,AVDD,ON);
      vfe_set_pmu_channel(sd,DVDD,ON);
      vfe_set_pmu_channel(sd,AFVDD,ON);
      //standby off io
      vfe_gpio_write(sd,PWDN,CSI_STBY_OFF);
      usleep_range(10000,12000);
      //reset after power on
      vfe_gpio_write(sd,RESET,CSI_RST_OFF);
      usleep_range(30000,31000);
      break;
    case CSI_SUBDEV_PWR_OFF:
      vfe_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
      //standby and reset io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      //reset on io
      vfe_gpio_write(sd,RESET,CSI_RST_ON);
      usleep_range(10000,12000);
      //power supply off
      vfe_gpio_write(sd,POWER_EN,CSI_PWR_OFF);
      vfe_set_pmu_channel(sd,AFVDD,OFF);
      vfe_set_pmu_channel(sd,DVDD,OFF);
      vfe_set_pmu_channel(sd,AVDD,OFF);
      vfe_set_pmu_channel(sd,IOVDD,OFF);
      //standby and reset io
      usleep_range(10000,12000);
      //inactive mclk after power off
      vfe_set_mclk(sd,OFF);
      //set the io to hi-z
      vfe_gpio_set_status(sd,RESET,0);//set the gpio to input
      vfe_gpio_set_status(sd,PWDN,0);//set the gpio to input
      break;
    default:
      return -EINVAL;
	}

	//remember to unlock i2c adapter, so the device can access the i2c bus again
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
	unsigned   int SENSOR_ID=0;
	unsigned char val;

//	ret = sensor_write(sd, 0xfe, 0x00);
//	if (ret < 0) {
//		vfe_dev_err("sensor_write err at sensor_detect!\n");
//		return ret;
//	}

	ret = sensor_read(sd, 0xfc, &val);
	SENSOR_ID|= (val<< 8);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}

	ret = sensor_read(sd, 0xfd, &val);
	SENSOR_ID|= (val);
	vfe_dev_print("V4L2_IDENT_SENSOR=%x",SENSOR_ID);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}

	if(SENSOR_ID != V4L2_IDENT_SENSOR)
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
	ret = sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
	msleep(350);
    return 0;
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
	{
		.desc		= "Raw RGB Bayer",
		.mbus_code	= V4L2_MBUS_FMT_SBGGR8_1X8,//linux-3.0
		.regs 		= sensor_fmt_raw,
		.regs_size = ARRAY_SIZE(sensor_fmt_raw),
		.bpp		= 1
	},
};
#define N_FMTS ARRAY_SIZE(sensor_formats)



/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */


static struct sensor_win_size
sensor_win_sizes[] = {
  /* UXGA */
  {
    .width      = UXGA_WIDTH,
    .height     = UXGA_HEIGHT,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_uxga_regs,
    .regs_size  = ARRAY_SIZE(sensor_uxga_regs),
    .set_size   = NULL,
  },
//  /* 720p */
//  {
//    .width      = HD720_WIDTH,
//    .height     = HD720_HEIGHT,
//    .hoffset    = 0,
//    .voffset    = 0,
//		.regs				= Gc2015_sensor_hd720_regs,
//		.regs_size	= ARRAY_SIZE(Gc2015_sensor_hd720_regs),
//    .set_size   = NULL,
//  },
  /* SVGA */
  {
    .width      = SVGA_WIDTH,
    .height     = SVGA_HEIGHT,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_svga_regs,
    .regs_size  = ARRAY_SIZE(sensor_svga_regs),
    .set_size   = NULL,
  },
  /* VGA */
  /*
  {
    .width      = VGA_WIDTH,
    .height     = VGA_HEIGHT,
    .hoffset    = 0,
    .voffset    = 0,
    .regs       = sensor_vga_regs,
    .regs_size  = ARRAY_SIZE(sensor_vga_regs),
    .set_size   = NULL,
  },
  */
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
             struct v4l2_mbus_framefmt *fmt)
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
	//unsigned  int  temp=0,shutter=0;
	//unsigned char val;

	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);

//	printk("chr wsize.width = [%d], wsize.height = [%d]\n", wsize->width, wsize->height);
	//vfe_dev_dbg("sensor_s_fmt\n");

	//////////////shutter-gain///////////////
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
	struct sensor_info *info = to_state(sd);

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;

	if (info->width > SVGA_WIDTH && info->height > SVGA_HEIGHT) {
		cp->timeperframe.denominator = SENSOR_FRAME_RATE/2;
	}
	else {
		cp->timeperframe.denominator = SENSOR_FRAME_RATE;
	}

	return 0;
}

static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
//	struct v4l2_captureparm *cp = &parms->parm.capture;
//	struct v4l2_fract *tpf = &cp->timeperframe;
//	struct sensor_info *info = to_state(sd);
//	int div;

//	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
//		return -EINVAL;
//	if (cp->extendedmode != 0)
//		return -EINVAL;

//	if (tpf->numerator == 0 || tpf->denominator == 0)
//		div = 1;  /* Reset to full rate */
//	else {
//		if (info->width > SVGA_WIDTH && info->height > SVGA_HEIGHT) {
//			div = (tpf->numerator*SENSOR_FRAME_RATE/2)/tpf->denominator;
//		}
//		else {
//			div = (tpf->numerator*SENSOR_FRAME_RATE)/tpf->denominator;
//		}
//	}
//
//	if (div == 0)
//		div = 1;
//	else if (div > 8)
//		div = 8;
//
//	switch()
//
//	info->clkrc = (info->clkrc & 0x80) | div;
//	tpf->numerator = 1;
//	tpf->denominator = sensor_FRAME_RATE/div;
//
//	sensor_write(sd, REG_CLKRC, info->clkrc);
	//return -EINVAL;
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

//  case V4L2_CID_3A_LOCK:
//    return v4l2_ctrl_query_fill(qc, 0, V4L2_LOCK_EXPOSURE |
//                                       V4L2_LOCK_WHITE_BALANCE |
//                                       V4L2_LOCK_FOCUS, 1, 0);
//  case V4L2_CID_AUTO_FOCUS_RANGE:
//    return v4l2_ctrl_query_fill(qc, 0, 0, 0, 0);//only auto
//  case V4L2_CID_AUTO_FOCUS_INIT:
//  case V4L2_CID_AUTO_FOCUS_RELEASE:
//  case V4L2_CID_AUTO_FOCUS_START:
//  case V4L2_CID_AUTO_FOCUS_STOP:
//  case V4L2_CID_AUTO_FOCUS_STATUS:
//    return v4l2_ctrl_query_fill(qc, 0, 0, 0, 0);
//  case V4L2_CID_FOCUS_AUTO:
//    return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//  case V4L2_CID_AUTO_EXPOSURE_WIN_NUM:
//    return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
//  case V4L2_CID_AUTO_FOCUS_WIN_NUM:
//    return v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
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
//  case V4L2_CID_POWER_LINE_FREQUENCY:
//    return sensor_g_band_filter(sd, &ctrl->value);

//  case V4L2_CID_3A_LOCK:
//  	return 0;
//  case V4L2_CID_AUTO_FOCUS_RANGE:
//  	ctrl->value=0;//only auto
//  	return 0;
//  case V4L2_CID_AUTO_FOCUS_INIT:
//  case V4L2_CID_AUTO_FOCUS_RELEASE:
//  case V4L2_CID_AUTO_FOCUS_START:
//  case V4L2_CID_AUTO_FOCUS_STOP:
//  case V4L2_CID_AUTO_FOCUS_STATUS:
//  	return 0;//sensor_g_af_status(sd);
////  case V4L2_CID_FOCUS_AUTO:
//  case V4L2_CID_AUTO_FOCUS_WIN_NUM:
//  	ctrl->value=1;
//  	return 0;
//  case V4L2_CID_AUTO_EXPOSURE_WIN_NUM:
//  	ctrl->value=1;
//  	return 0;
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
//    case V4L2_CID_POWER_LINE_FREQUENCY:
//      return sensor_s_band_filter(sd,
//          (enum v4l2_power_line_frequency) ctrl->value);

//    case V4L2_CID_3A_LOCK:
//    	return 0;//sensor_s_3a(sd, ctrl->value);
//    case V4L2_CID_AUTO_FOCUS_RANGE:
//  	  return 0;
//	  case V4L2_CID_AUTO_FOCUS_INIT:
//	  	return sensor_s_init_af(sd);
//	  case V4L2_CID_AUTO_FOCUS_RELEASE:
//	  	return sensor_s_release_af(sd);
//	  case V4L2_CID_AUTO_FOCUS_START:
//	  	return sensor_s_single_af(sd);
//	  case V4L2_CID_AUTO_FOCUS_STOP:
//	  	return sensor_s_pause_af(sd);
//	  case V4L2_CID_AUTO_FOCUS_STATUS:
//	    return 0;
//	  case V4L2_CID_FOCUS_AUTO:
//	  	return sensor_s_continueous_af(sd, ctrl->value);
//	  case V4L2_CID_AUTO_FOCUS_WIN_NUM:
//	  	vfe_dev_dbg("s_ctrl win value=%d\n",ctrl->value);
//	  	return sensor_s_af_zone(sd, (struct v4l2_win_coordinate *)(ctrl->user_pt));
//	  case V4L2_CID_AUTO_EXPOSURE_WIN_NUM:
//	  	return 0;
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
	v4l2_i2c_subdev_init(sd, client, &sensor_ops);

	client->addr=0xDE>>1;

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
//	info->clkrc = 1;	/* 30fps */

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "bf3920", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
	.name = "bf3920",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return i2c_add_driver(&sensor_driver);
}

static __exit void exit_sensor(void)
{
  i2c_del_driver(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
