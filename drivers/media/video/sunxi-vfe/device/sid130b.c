/*
 * A V4L2 driver for SET SID130B cameras.
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
#include <media/v4l2-mediabus.h>//linux-3.0
#include <linux/io.h>


#include "camera.h"


MODULE_AUTHOR("raymonxiu");
MODULE_DESCRIPTION("A low-level driver for GalaxyCore sid130b sensors");
MODULE_LICENSE("GPL");

//for internel driver debug
#define DEV_DBG_EN   		1 
#if(DEV_DBG_EN == 1)		
#define vfe_dev_dbg(x,arg...) printk("[CSI_DEBUG][SID130B]"x,##arg)
#else
#define vfe_dev_dbg(x,arg...) 
#endif
#define vfe_dev_err(x,arg...) printk("[CSI_ERR][SID130B]"x,##arg)
#define vfe_dev_print(x,arg...) printk("[CSI][SID130B]"x,##arg)

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
#define VREF_POL	 V4L2_MBUS_VSYNC_ACTIVE_HIGH
#define HREF_POL	 V4L2_MBUS_HSYNC_ACTIVE_HIGH
#define CLK_POL		V4L2_MBUS_PCLK_SAMPLE_RISING
#define V4L2_IDENT_SENSOR 0x130B

//define the voltage level of control signal
#define CSI_STBY_ON			0
#define CSI_STBY_OFF 		1
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0

#define regval_list reg_list_a8_d8
#define REG_TERM 0xff
#define VAL_TERM 0xff
#define REG_DLY  0xffff


/*
 * Our nominal (default) frame rate.
 */
#define SENSOR_FRAME_RATE 8



/*
 * Our nominal (default) frame rate.
 */
#define I2C_ADDR 0x6E

/* Registers */


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

	{{0x00},{0x00}}, 	//snr block 
    {{0x04},{0x03}},    //Group B
	{{0x05},{0x0F}},  //UXGA Output
	{{0x06},{0x0f}},  //gb-preview
    {{0x07},{0x08}}, 

	{{0x08},{0xa2}},  //0xa2 pll off
	{{0x09},{0x11}}, 	//ga-still 24-32 
    {{0x0A},{0x11}},    
	{{0x10},{0x11}},//13}}, 	 
	{{0x11},{0x01}},  
	{{0x12},{0x8A}},  
	{{0x13},{0x1a}}, 
	{{0x14},{0x27}},  
	{{0x15},{0x22}},  

	{{0x17},{0xCb}}, 	 
	{{0x18},{0x38}}, 	 

	{{0x20},{0x00}}, //0x00
	{{0x21},{0x02}}, //0x02
	{{0x23},{0x51}}, //0x51

	//gb-preview 
	{{0x24},{0x00}}, //0x00
	{{0x25},{0x02}}, //0x10
	{{0x27},{0x51}}, //0x4b






	{{0x40},{0x0f}},  
	{{0x41},{0x17}},  
	{{0x42},{0x52}},  
	{{0x43},{0x80}},  
	{{0x44},{0x00}},  
	{{0x45},{0x00}},  



	{{0x00},{0x01}},  //ae block 
	{{0x10},{0x80}}, 
	{{0x11},{0x10}},  
	{{0x12},{0x78}},  //0x78
	{{0x13},{0x78}},  //0x78
	{{0x14},{0x78}},  //0x70
	{{0x17},{0xc4}}, 
	//{{0x1e},{0x00}}, 

	{{0x34},{0x78}}, //0x64 //gb-preview
	{{0x35},{0x78}}, //0x32 //ga-still 
	{{0x36},{0x40}}, //0x26


	{{0x40},{0x40}},  //40

	//gain tabe   							
	{{0x41},{0x28}}, 
	{{0x42},{0x20}}, 
	{{0x43},{0x00}}, 
	{{0x44},{0x00}}, 
	{{0x45},{0x01}}, 
	{{0x46},{0x03}}, 
	{{0x47},{0x05}}, 
	{{0x48},{0x0a}}, 
	{{0x49},{0x0e}}, 
	{{0x4a},{0x13}}, 
	{{0x4b},{0x18}}, 
	{{0x4c},{0x1a}}, 
	{{0x4d},{0x1e}},
	{{0x4e},{0x0f}}, 
	{{0x4f},{0x09}}, 
	{{0x50},{0x07}}, 
	{{0x51},{0x05}}, 
	{{0x52},{0x04}}, 
	{{0x53},{0x03}}, 
	{{0x54},{0x02}}, 
	{{0x55},{0x01}}, 

	{{0xc0},{0x11}}, 
	{{0xc1},{0x11}}, 
	{{0xc2},{0x11}}, 
	{{0xc3},{0x01}}, 
	{{0xc4},{0x0f}},
//    {{0x90},{0xe2}}, 
//    {{0x92},{0x0c}},  //0bank 0x11 苞 悼老 
//    {{0x95},{0x40}}, 	//0 bank 0x40 苞 悼老  



    {{0x00},{0x02}},

    {{0x10},{0xd0}},	// 0xD3
    {{0x11},{0x11}},	// 0x11
    {{0x13},{0x80}},	// 0x7e
    {{0x14},{0x80}},	// 0x7e
    {{0x15},{0xd0}},	// 0xee
    {{0x16},{0x80}},	// 0x80
    {{0x17},{0xd0}},	// 0xd0
    {{0x18},{0x78}},	// 0x80
    {{0x19},{0x98}},	// 0x98
    {{0x1A},{0x68}},	// 0x68
    {{0x1B},{0x98}},	// 0x98
    {{0x1C},{0x68}},	// 0x68
    {{0x1D},{0x90}},	// 0xa8
    {{0x1E},{0x74}},	// 0x70

    {{0x20},{0xF0}},	// 0xf0
    {{0x21},{0x70}},	// 0x98
    {{0x22},{0xa4}},	// 0xb4
    {{0x23},{0x20}},	// 0x20
    {{0x25},{0x20}},	// 0x20
    {{0x26},{0x05}},	// 0x05
    {{0x27},{0x00}},	// 0x08
    {{0x28},{0x00}},	// 0x08
    {{0x29},{0xa1}},	// 0xb9
    {{0x2A},{0x9a}},	// 0x9d

	{{0x30},{0x00}},  
	{{0x31},{0x10}},  
	{{0x32},{0x00}},  
	{{0x33},{0x10}},  
	{{0x34},{0x06}},  
	{{0x35},{0x30}},  
	{{0x36},{0x04}},  
	{{0x37},{0xA0}},  
	{{0x40},{0x01}},  
	{{0x41},{0x04}},  
	{{0x42},{0x08}},  
	{{0x43},{0x10}},  
	{{0x44},{0x13}},  
    {{0x45},{0x64}},	// 0x6B
	{{0x46},{0x82}},  
    {{0x50},{0xd0}},	// 0x13
    {{0x51},{0x80}},	// 0x6B
    {{0x52},{0x82}},	// 0x82

    // CMA change  -D65~A
    {{0x53},{0x97}},	// AWB R Gain for D30 to D20	0xa3
    {{0x54},{0xaf}},	// AWB B Gain for D30 to D20	0xb6
    {{0x55},{0x97}},	// AWB R Gain for D20 to D30	0xa3
    {{0x56},{0xaf}},	// AWB B Gain for D20 to D30	0xb6
    {{0x57},{0xB5}},	// AWB R Gain for D65 to D30	0xc5
    {{0x58},{0x8f}},	// AWB B Gain for D65 to D30	0x99
    {{0x59},{0xB5}},	// AWB R Gain for D30 to D65	0xc5
    {{0x5A},{0x8f}},	// AWB B Gain for D30 to D65	0x99

	{{0xB4},{0x05}}, 
	{{0xB5},{0x0F}},   
	{{0xB6},{0x06}},   
	{{0xB7},{0x06}},   
	{{0xB8},{0x40}},   
	{{0xB9},{0x10}},   
	{{0xBA},{0x06}},   


	{{0x00},{0x03}},  //idp3 block
	{{0x10},{0xFF}},  
    {{0x11},{0x19}}, 
    {{0x12},{0x3d}},
    {{0x13},{0xff}},	//FF 
    {{0x14},{0x00}}, 
    {{0x15},{0xc0}},	//c0 

	//dpc         
	{{0x30},{0x88}},  
	{{0x31},{0x14}},  
	{{0x32},{0x08}},  
	{{0x33},{0x06}},  
	{{0x34},{0x04}},  
	{{0x35},{0x04}},  
	{{0x36},{0x44}},  
	{{0x37},{0x66}}, 

	{{0x38},{0x01}},  
	{{0x39},{0x02}},  
	{{0x3A},{0x04}},  
	{{0x3B},{0x20}},  
	{{0x3C},{0x20}},  
    {{0x3D},{0x01}},   //NRTHVRNG0 @ AGAIN = 00 0x2 0x02 0420
    {{0x3E},{0x02}},   //NRTHVRNG1 @ AGAIN = 20 0x4 0x04 0420
    {{0x3F},{0x4}},   //NRTHVRNG2 @ AGAIN = 40 0x4 0x06 0420
    {{0x40},{0x10}},   //NRTHVRNG3 @ AGAIN = 60 0x8 0x08 0420
    {{0x41},{0x20}},   //NRTHVRNG4 @ AGAIN = 80 0x8 0x40 0420
    {{0x42},{0xff}},   //NRTHVRNGMAX
    {{0x43},{0x10}},   //NRTHRWGT
    {{0x44},{0x40}},   //BASELVL
    {{0x45},{0x06}},   //SHUMAXH
    {{0x46},{0x40}},   //SHUMAXL
    {{0x47},{0x1e}},   //ILLUMITHDRK

	// shading
	{{0x50},{0x00}}, 	
	{{0x51},{0x00}}, 	
	{{0x52},{0x00}}, 	
	{{0x53},{0x00}}, 	

	{{0x54},{0x00}}, 	
	{{0x55},{0x00}}, 	
	{{0x56},{0x00}}, 	
	{{0x57},{0x00}}, 	

	{{0x58},{0x00}}, 	
	{{0x59},{0x00}}, 	
    {{0x5A},{0x00}}, //0x04	//0x10  //20 //26                   
    {{0x5B},{0x00}}, //0x14	//0x20  //15 //02 
                   
    {{0x5C},{0x00}}, //0x04	//0x04  //33 //26 //1                
    {{0x5D},{0x00}}, //0x10	//0x55  //42 //01               
    {{0x5E},{0x00}}, //0x04	//0x25  //49 //02  38             
    {{0x5F},{0x00}}, //0x00	//0x20  //30 //02 
    
                 
    {{0x6B},{0x00}}, //0x00 //01 //01                 
    {{0x6C},{0x00}}, //0x11 //22 :                  
    {{0x6D},{0x43}}, //0x33 //22                   
    {{0x6E},{0x43}}, //0x44 //55                   
    {{0x6F},{0x43}}, //0x45 //77                   
    {{0x70},{0x21}}, //0x66 //65  
            
    {{0x71},{0x00}}, //0x00 //01 //AB (6)               
    {{0x72},{0x00}}, //0x12 //23     (5)             
    {{0x73},{0x43}}, //0x23 //33     (4)            
    {{0x74},{0x43}}, //0x44 //45     (3)             
    {{0x75},{0x43}}, //0x45 //55     (2)               
    {{0x76},{0x21}}, //0x66 //55     (1)             

    {{0x77},{0x00}}, //0x00 //01 //AB (6)                 
    {{0x78},{0x00}}, //0x12 //23     (5)              
    {{0x79},{0x43}}, //0x23 //33     (4)              
    {{0x7A},{0x43}}, //0x44 //45     (3)              
    {{0x7B},{0x43}}, //0x45 //55     (2)                
    {{0x7C},{0x21}}, //0x66 //55     (1)              

    {{0x7D},{0x00}}, //0x00 //00    //00                   
    {{0x7E},{0x00}}, //0x11 //00    //00                   
    {{0x7F},{0x32}}, //0x12 //11    //12                   
    {{0x80},{0x32}}, //0x33 //33    //33 //44               
    {{0x81},{0x32}}, //0x44 //33    //33 //44                  
    {{0x82},{0x10}}, //0x55 //22    //44 //45 
 //   {{0x83},{0x15}},
 //   {{0x84},{0x32}},

	{{0x94},{0x06}}, 
	{{0x95},{0x40}}, 
	{{0x96},{0x04}}, 
	{{0x97},{0xb0}}, 

    //Interpolation
    {{0xA0},{0x2F}},
    {{0xA1},{0x02}},
    {{0xA2},{0xB7}},
    {{0xA3},{0xB7}},
    {{0xA4},{0x08}},   //capture 08 0420
    {{0xA5},{0xFF}},
    {{0xA6},{0x06}},	 //capture 08 0420
    {{0xA7},{0xFF}},
    {{0xA8},{0x00}},
    {{0xA9},{0x00}},
    {{0xAA},{0x00}},
    {{0xAB},{0x00}},
    {{0xAC},{0x60}},
    {{0xAD},{0x18}},
    {{0xAE},{0x10}},	//capture 08 0420
    {{0xAF},{0x20}},	//capture 14 0420
    {{0xB0},{0x08}},
    {{0xB1},{0x00}},

    //Color Matrix for D65
     {{0xC0},{0xAF}},//0x2F}}, //// CMASB D20 or D30 or Dark Condition Color Matrix Selection
     {{0xC1},{0x6a}}, //
     {{0xC2},{0xce}}, //
     {{0xC3},{0x07}}, //
     {{0xC4},{0xe8}}, //
     {{0xC5},{0x64}}, //
     {{0xC6},{0xf3}}, //
     {{0xC7},{0xF7}}, //
     {{0xC8},{0xCb}}, //
     {{0xC9},{0x7d}}, //
     {{0xCA},{0xd1}}, //
     {{0xCB},{0x19}}, //
     {{0xCC},{0x16}}, //
     {{0xCD},{0x42}}, //
     {{0xCE},{0x0b}}, //

    //Color Matrix for CWF
    {{0xD0},{0xAF}},
    {{0xD1},{0x71}},
    {{0xD2},{0xce}},
    {{0xD3},{0x00}},
    {{0xD4},{0xe8}},
    {{0xD5},{0x67}},
    {{0xD6},{0xf0}},
    {{0xD7},{0xF4}},
    {{0xD8},{0xCb}},
    {{0xD9},{0x80}},
    {{0xDA},{0x41}},
    {{0xDB},{0xb9}},
    {{0xDC},{0x16}},
    {{0xDD},{0x42}},
    {{0xDE},{0x0b}},

    //Color Matrix for A
    {{0xE0},{0x2F}}, 
    {{0xE1},{0x6B}}, 
    {{0xE2},{0xC7}}, 
    {{0xE3},{0x0D}}, 
    {{0xE4},{0xE3}}, 
    {{0xE5},{0x61}}, 
    {{0xE6},{0xFA}}, 
    {{0xE7},{0xE5}}, 
    {{0xE8},{0x98}}, 
    {{0xE9},{0xC1}}, 
    {{0xEA},{0xC4}}, 
    {{0xEB},{0x04}}, 
    {{0xEC},{0xEE}}, 
    {{0xED},{0xDA}}, 
    {{0xEE},{0x09}}, 



	{{0x00},{0x04}},    //idp4 block
	//gamma -  aa   
	{{0x10},{0x00}},   
	{{0x11},{0x0a}},   
	{{0x12},{0x13}},  
	{{0x13},{0x25}},  
	{{0x14},{0x48}},  
	{{0x15},{0x64}},  
	{{0x16},{0x7c}},  
	{{0x17},{0x8e}},  
	{{0x18},{0xa0}},  
	{{0x19},{0xae}},  
	{{0x1A},{0xba}},  
	{{0x1B},{0xcf}},  
	{{0x1C},{0xe0}},  
	{{0x1D},{0xf0}},  
	{{0x1E},{0xf8}},  
	{{0x1F},{0xFF}},  
	//gamma -  aa 			
	{{0x20},{0x00}},   
	{{0x21},{0x0a}},   
	{{0x22},{0x13}},  
	{{0x23},{0x25}},  
	{{0x24},{0x48}},  
	{{0x25},{0x64}},  
	{{0x26},{0x7c}},  
	{{0x27},{0x8e}},  
	{{0x28},{0xa0}},  
	{{0x29},{0xae}},  
	{{0x2A},{0xba}},  
	{{0x2B},{0xcf}},  
	{{0x2C},{0xe0}},  
	{{0x2D},{0xf0}},  
	{{0x2E},{0xf8}},  
	{{0x2F},{0xFF}}, 
	//gamma -  aa 			 
	{{0x30},{0x00}},   
	{{0x31},{0x0a}},   
	{{0x32},{0x13}},  
	{{0x33},{0x25}},  
	{{0x34},{0x48}},  
	{{0x35},{0x64}},  
	{{0x36},{0x7c}},  
	{{0x37},{0x8e}},  
	{{0x38},{0xa0}},  
	{{0x39},{0xae}},  
	{{0x3A},{0xba}},  
	{{0x3B},{0xcf}},  
	{{0x3C},{0xe0}},  
	{{0x3D},{0xf0}},  
	{{0x3E},{0xf8}},  
	{{0x3F},{0xFF}},   

	{{0x40},{0x00}},   
	{{0x41},{0x0a}},   
	{{0x42},{0x13}},  
	{{0x43},{0x25}},  
	{{0x44},{0x48}},  
	{{0x45},{0x64}},  
	{{0x46},{0x7c}},  
	{{0x47},{0x8e}},  
	{{0x48},{0xa0}},  
	{{0x49},{0xae}},  
	{{0x4A},{0xba}},  
	{{0x4B},{0xcf}},  
	{{0x4C},{0xe0}},  
	{{0x4D},{0xf0}},  
	{{0x4E},{0xf8}},  
	{{0x4F},{0xFF}},  
	//csc																	   
	{{0x60},{0x33}},  
	{{0x61},{0x20}},  
	{{0x62},{0xE4}},    
	{{0x63},{0xFA}},    
	{{0x64},{0x13}},    
	{{0x65},{0x25}},    
	{{0x66},{0x07}},    
	{{0x67},{0xF5}},    
	{{0x68},{0xEA}},    
	{{0x69},{0x20}},    
	{{0x6A},{0xC8}},    
	{{0x6B},{0xC4}},    
	{{0x6C},{0x84}},    
	{{0x6D},{0x04}},    
	{{0x6E},{0x0C}},    
	{{0x6F},{0x00}},   

	//edge						
	{{0x70},{0x00}},  
	{{0x71},{0x00}},  
	{{0x72},{0x00}},  
	{{0x73},{0x00}},  
	{{0x74},{0x00}},  
	{{0x75},{0x00}},  
	{{0x77},{0x00}}, 
	{{0x78},{0x00}}, 
	{{0x79},{0x00}}, 
	{{0x7A},{0x00}},    
	{{0x7B},{0x00}},    
	{{0x7C},{0x00}},    
	{{0x7D},{0x00}},    
	{{0x7E},{0x00}},    
	{{0x7F},{0x00}},   

	{{0x80},{0x22}},   
    {{0x81},{0x04}}, //20	//eugain
    {{0x82},{0x08}}, //28	//edgain 
    {{0x83},{0x04}}, //coring
	//{{0x84},{0x20}},   
//    {{0x85},{0x05}},	//max
    {{0x86},{0x10}}, 		//eucorslop
    {{0x87},{0x04}}, //coring
	//{{0x88},{0x20}},  
    {{0x89},{0x05}},	//max
	//{{0x8a},{0x02}}, 	
	//{{0x8b},{0x30}},  
	//{{0x8c},{0x30}},  
    {{0x91},{0x03}},
    {{0x93},{0x60}},

    //Cr/Cb Coring
    {{0x94},{0x00}}, 
    {{0x95},{0x00}}, 
    {{0x96},{0x4C}}, 
    {{0x97},{0x66}}, 	// edge
    {{0x9A},{0xfa}}, 
    {{0xA1},{0x08}},     //@ 0
    {{0xA2},{0x10}},     //@ 20
    {{0xA3},{0x16}},     //@ 40
    {{0xA4},{0x20}},     //@ 60
    {{0xA5},{0x30}},     //@ 80
    {{0xA6},{0xa0}}, 
    {{0xA7},{0x06}}, 
    {{0xA8},{0x40}}, 
    {{0xA9},{0x20}}, 
    {{0xAa},{0x28}}, 
    {{0xAc},{0xff}}, 
    {{0xAd},{0x09}}, 
    {{0xAe},{0x96}}, 
    {{0xAf},{0x18}}, 
    {{0xB2},{0x24}},     //color suppression start
    {{0xB3},{0x11}}, 
    {{0xB6},{0x00}},	//0420 

    //Color Saturation
    {{0xBa},{0x0f}}, 
    {{0xBb},{0x0f}}, 
    {{0xBC},{0x0f}}, 
    {{0xBD},{0x0f}}, 
    {{0xBE},{0x0f}}, 
    {{0xBF},{0x0f}}, 
    {{0xc0},{0x10}},
    {{0xc1},{0x10}}, 	
    {{0xc2},{0x10}}, 
    {{0xc3},{0x10}}, 
    {{0xc4},{0x10}}, 
    {{0xc5},{0x10}},

    {{0xc6},{0x01}},
    {{0xc7},{0x01}},
    {{0xc8},{0x01}},
    {{0xc9},{0x01}},
    {{0xca},{0x01}},
    {{0xcb},{0x01}},

    {{0xcc},{0x04}}, 
    {{0xcd},{0x3f}}, 
    {{0xce},{0x81}}, 

    {{0xE0},{0x00}}, 
    {{0xE1},{0x00}}, 
    {{0xE2},{0x00}}, 
    {{0xE3},{0x00}}, 	
    {{0xE4},{0x00}}, 
    {{0xE5},{0x00}}, 
    {{0xE6},{0x00}}, 
    {{0xE7},{0x00}}, 
    {{0xE8},{0x00}}, 	




	{{0x00},{0x05}},  //idp5 block
	{{0x10},{0x00}}, 	 
	{{0x11},{0x00}}, 	 
	{{0x12},{0x00}}, 	 
	{{0x13},{0x00}}, 	 
	{{0x14},{0x00}}, 	 
	{{0x15},{0x00}}, 	 
	{{0x16},{0x00}},  
	{{0x17},{0x00}}, 	 
	{{0x18},{0x00}},  

	{{0x30},{0x00}}, 	 
	{{0x31},{0x00}}, 	 
	{{0x32},{0x00}}, 	 
	{{0x33},{0x00}}, 	 

	//memorymm 			
	{{0x40},{0x15}},  
	{{0x41},{0x28}},  
	{{0x42},{0x04}},  
	{{0x43},{0x15}},  
	{{0x44},{0x28}},  
	{{0x45},{0x04}},  
	{{0x46},{0x15}},  
	{{0x47},{0x28}},  
	{{0x48},{0x04}},  

	//knee						
	{{0x90},{0x00}}, 	 
	{{0x91},{0x00}}, 	 
	{{0x92},{0x00}}, 	 
	{{0x93},{0x00}}, 	 
	{{0x94},{0x00}}, 	 
	{{0x95},{0x00}}, 	 
	{{0x96},{0x00}},  

    //ADG                   
    {{0x99},{0x00}},    
    {{0xA0},{0x00}},    
    {{0xA1},{0x00}},    
    {{0xA2},{0x00}},    
    {{0xA3},{0x00}},    
    {{0xA4},{0x00}},    
    {{0xA5},{0x00}}, 
    {{0xA6},{0x00}}, 
    {{0xA7},{0x00}}, 
    {{0xA8},{0x00}}, 
    {{0xA9},{0x00}}, 
    {{0xAA},{0x00}}, 
    {{0xAB},{0x00}},    
    {{0xAC},{0x00}},    
    {{0xAD},{0x00}},    
    {{0xAE},{0x00}},    
    {{0xAF},{0x00}},     

    //YXGMA                 
    {{0xB0},{0x00}},    
    {{0xB1},{0x00}},    
    {{0xB8},{0x00}},    
    {{0xB9},{0x00}},    
    {{0xBA},{0x00}},    
    {{0xBB},{0x00}},    
	{{0xB0},{0xf0}}, 	
	{{0xB5},{0x0f}}, 

	{{0xC0},{0x00}},  
	{{0xC1},{0x03}},  
	{{0xC2},{0x06}},  
	{{0xC3},{0x0d}},  
	{{0xC4},{0x1d}},  
	{{0xC5},{0x2e}},  
	{{0xC6},{0x3e}}, 	 
	{{0xC7},{0x4f}}, 	 
	{{0xC8},{0x60}}, 	 
	{{0xC9},{0x70}}, 	 
	{{0xCA},{0x80}}, 	 
	{{0xCB},{0xa0}}, 	 
	{{0xCC},{0xc0}},  
	{{0xCD},{0xe0}},  
	{{0xCE},{0xf0}},  
	{{0xCF},{0xff}},  

    // edge value adjustment
    {{0xe0},{0x01}},
    {{0xe1},{0x03}},
    {{0xe2},{0x04}},
    {{0xe3},{0x0c}},
    {{0xe4},{0x11}},
    {{0xe5},{0x16}},
    {{0xe6},{0x1b}},
    {{0xe7},{0x24}},
    {{0xe8},{0x30}},

	{{0x00},{0x00}},  //snr block
	//sensor o oo 
    {{0x03},{0x55}}, 
  
};


static struct regval_list sensor_uxga_regs[] = {

	{{0x00},{0x00}}, //snr block
    {{0x04},{ 0x13}},//{{0x04},{ 0x13}},
	{{0x05},{0x0f}},	//ga-still
	{{0x06},{0x0f}},	//gb-preview

	//ga-still 
	{{0x20},{0x00}}, //0x00
	{{0x21},{0x02}}, //0x02
	{{0x23},{0x15}}, //0x51

	//gb-preview 
	{{0x24},{0x00}}, //0x00
	{{0x25},{0x02}}, //0x10
	{{0x27},{0x15}}, //0x4b

	{{0x41},{ 0x17}},
	{{0x42},{ 0x52}},


	{{0x00},{0x01}}, //ae block
	{{0x34},{0x78}}, //0x64 //gb-preview
	{{0x35},{0x78}}, //0x32 //ga-still 
	{{0x36},{0x40}}, //0x26



	{{0x00},{0x03}}, //idp3 block
	{{0x94},{0x06}}, //win size 1600x1200
	{{0x95},{0x40}},
	{{0x96},{0x04}},
	{{0x97},{0xb0}},

	{{0x00},{0x04}},
	{{0x81},{0x04}}, //20	//eugain
    {{0x82},{0x08}}, //28	//edgain 

	//scaler     
    {{0xe0},{ 0x00}},

        // edge value adjustment
    {{0x00},{0x05}},
    {{0xe0},{0x01}},
    {{0xe1},{0x03}},
    {{0xe2},{0x04}},
    {{0xe3},{0x0c}},
    {{0xe4},{0x11}},
    {{0xe5},{0x16}},
    {{0xe6},{0x1b}},
    {{0xe7},{0x24}},
    {{0xe8},{0x30}},
	
	
	//banding
	
};
	/* 800X600 SVGA*/
static struct regval_list sensor_svga_regs[] = {
	//Resolution Setting : 800*600

	{{0x00},{0x00}}, //snr block
        {{0x04},{0x13}}, 
	{{0x05},{0x0f}},	//ga-still
	{{0x06},{0x0f}},	//gb-preview

	//ga-still 
	{{0x20},{0x00}}, //0x00
	{{0x21},{0x02}}, //0x02
	{{0x23},{0x15}}, //0x51

	//gb-preview 
	{{0x24},{0x00}}, //0x00
	{{0x25},{0x02}}, //0x10
	{{0x27},{0x15}}, //0x4b



	{{0x00},{0x01}}, //ae block
	{{0x34},{0x78}}, //0x64 //gb-preview
	{{0x35},{0x78}}, //0x32 //ga-still 
	{{0x36},{0x40}}, //0x26





	{{0x00},{0x04}},
	{{0x81},{0x04}}, //20	//eugain
	{{0x82},{0x08}}, //28	//edgain 

	//scaler	       

        {{0xE0},{0xe0}}, 
        {{0xE1},{0x80}}, 
        {{0xE2},{0x80}}, 
        {{0xE3},{0x00}},	
        {{0xE4},{0x00}}, 
        {{0xE5},{0x03}}, 
        {{0xE6},{0x25}}, 
        {{0xE7},{0x03}}, 
        {{0xE8},{0x20}},	

};

	/* 640X480 VGA */
static struct regval_list sensor_vga_regs[] = {
	//Resolution Setting : 640X480 预览走这里
	//scaler	       
	{{0x00},{0x00}}, //snr block
	{{0x04},{0x03}}, //gb-selset
	{{0x05},{0x0f}},	//ga-still
	{{0x06},{0x0f}},	//gb-preview

	//ga-still 
	{{0x20},{0x00}}, //0x00
	{{0x21},{0x02}}, //0x02
	{{0x23},{0x15}}, //0x51

	//gb-preview 
	{{0x24},{0x00}}, //0x00
	{{0x25},{0x02}}, //0x10
	{{0x27},{0x15}}, //0x4b

	{{0x41},{ 0x17}},
	{{0x42},{ 0x52}},

	{{0x00},{0x01}}, //ae block
	{{0x34},{0x78}}, //0x64 //gb-preview
	{{0x35},{0x78}}, //0x32 //ga-still 
	{{0x36},{0x40}}, //0x26





	{{0x00},{0x04}},
	{{0x81},{0x04}}, //20	//eugain
	{{0x82},{0x08}}, //28	//edgain 
	{{0xe0},{ 0xe0}},
	{{0xe1},{ 0x66}},
	{{0xe2},{ 0x66}},
	{{0xe4},{ 0x00}},
	{{0xe5},{ 0x03}},
	{{0xe6},{ 0xc5}},
	{{0xe7},{ 0x02}},
	{{0xe8},{ 0x80}},

	// edge value adjustment
	{{0x00},{ 0x05}},
	{{0xe0},{0x81}},
	{{0xe1},{0x83}},
	{{0xe2},{0x04}},
	{{0xe3},{0x0c}},
	{{0xe4},{0x14}},
	{{0xe5},{0x1c}},
	{{0xe6},{0x3f}},
	{{0xe7},{0x46}},
	{{0xe8},{0x52}},

		
};



/*
 * The white balance settings
 * Here only tune the R G B channel gain. 
 * The white balance enalbe bit is modified in sensor_s_autowb and sensor_s_wb
 */
static struct regval_list sensor_wb_auto_regs[] = {
	{{0x00},{0x02}}, 
	//{{0x50},{0xa8}},
//	{{0x51},{0xa8}},
	{{0x10},{0xD0}}
};

static struct regval_list sensor_wb_manual[] = { 
//null

};
/*
static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
	{{0x00},{0x02}}, 
	{{0x50},{0xc2}},
	{{0x51},{0x9e}},
	{{0x10},{0xD0}}
};
*/
static struct regval_list sensor_wb_incandescence_regs[] = {
	{{0x00},{0x02}}, 
	{{0x50},{0x98}},
	{{0x51},{0xc8}},
	{{0x10},{0xD0}}
};

static struct regval_list sensor_wb_fluorescent_regs[] = {
	//tai yang guang
	{{0x00},{0x02}}, 
	{{0x50},{0xaa}},
	{{0x51},{0xbe}},
	{{0x10},{0xD0}}
};

static struct regval_list sensor_wb_tungsten_regs[] = {
	//wu si deng
	{{0x00},{0x02}}, 
	{{0x50},{0x90}},
{{0x51},{0xc0}},
	{{0x10},{0xD0}}
};

static struct regval_list sensor_wb_horizon[] = { 
//null
};
static struct regval_list sensor_wb_daylight_regs[] = {
	//tai yang guang
//null

};

static struct regval_list sensor_wb_flash[] = { 
//null
};


static struct regval_list sensor_wb_cloud_regs[] = {
//null

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
	{{0x00},{0x04}},
	{{0xd9},{0x00}},
};

static struct regval_list sensor_colorfx_bw_regs[] = {
       {{0x00},{0x04}}, 
	{{0xd9},{0x40}},
//	{{0xda},{0xc0}},
//	{{0xdb},{0xD0}}
	
};

static struct regval_list sensor_colorfx_sepia_regs[] = {
       {{0x00},{0x04}}, 
	{{0xd9},{0x80}},
	{{0xda},{0x60}},
	{{0xdb},{0xa0}}
	
};

static struct regval_list sensor_colorfx_negative_regs[] = {
     
	
};

static struct regval_list sensor_colorfx_emboss_regs[] = {
       {{0x00},{0x04}}, 
	{{0xd9},{0x08}},
//	{{0xda},{0xc0}},
//	{{0xdb},{0xD0}}
//NULL
};

static struct regval_list sensor_colorfx_sketch_regs[] = {
         {{0x00},{0x04}}, 
	{{0xd9},{0x04}},
	//{{0xda},{0xc0}},
	//{{0xdb},{0xD0}}
//NULL
};

static struct regval_list sensor_colorfx_sky_blue_regs[] = {
          {{0x00},{0x04}}, 
	{{0xd9},{0x80}},
	{{0xda},{0xc0}},
	{{0xdb},{0x60}}
	
};

static struct regval_list sensor_colorfx_grass_green_regs[] = {
{{0x00},{0x04}}, 
	{{0xd9},{0x80}},
	{{0xda},{0x50}},
	{{0xdb},{0x50}}
	
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
	{{0x00},{0x04}},  
	{{0xb6},{0xc0}}
};

static struct regval_list sensor_ev_neg3_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0xb0}}
};

static struct regval_list sensor_ev_neg2_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0xa0}}
};

static struct regval_list sensor_ev_neg1_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0x90}}
};                     

static struct regval_list sensor_ev_zero_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0x28}},//18}} //10
};

static struct regval_list sensor_ev_pos1_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0x35}},//28}}
};

static struct regval_list sensor_ev_pos2_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0x40}},//38}}
};

static struct regval_list sensor_ev_pos3_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0x50}},//48}}
};

static struct regval_list sensor_ev_pos4_regs[] = {
	{{0x00},{0x04}},  
	{{0xb6},{0x60}},//58}}
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
	
	{{0x00},{0x03}},
	{{0x12},{0x3d}}	//YCbYCr
};


static struct regval_list sensor_fmt_yuv422_yvyu[] = {
	
	{{0x00},{0x03}},
	{{0x12},{0x2d}}	//YCrYCb
};

static struct regval_list sensor_fmt_yuv422_vyuy[] = {
	
	{{0x00},{0x03}},
	{{0x12},{0x0d}}	//CrYCbY
};

static struct regval_list sensor_fmt_yuv422_uyvy[] = {
	
	{{0x00},{0x03}},
	{{0x12},{0x1d}}	//CbYCrY
};

/*static struct regval_list sensor_fmt_raw[] = {
	{0x24,0xb7},//raw
};
*/


/*
 * Low-level register I/O.
 *
 */


/*
 * On most platforms, we'd rather do straight i2c I/O.
 */
static int sensor_read(struct v4l2_subdev *sd, unsigned char reg,
		unsigned char *value)
{
	int ret=0;
	int cnt=0;
	
  //struct i2c_client *client = v4l2_get_subdevdata(sd);
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
	
  //struct i2c_client *client = v4l2_get_subdevdata(sd);
  
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
  	return 0;
  while(i<array_size)
  {
    if(regs->addr == REG_DLY) {
      msleep(regs->data);
	}
	else {
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
	
	ret = sensor_write(sd, 0x00, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_hflip!\n");
		return ret;
	}
	
	ret = sensor_read(sd, 0x04, &val);
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
	
	ret = sensor_write(sd, 0x00, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_hflip!\n");
		return ret;
	}
	
	ret = sensor_read(sd, 0x04, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_hflip!\n");
		return ret;
	}
	
	switch (value) {
		case 0:
		  val &= 0x00;
			break;
		case 1:
			val |= 0x01;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, 0x04, val);
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
	
	ret = sensor_write(sd, 0x00, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_vflip!\n");
		return ret;
	}
	
	ret = sensor_read(sd, 0x04, &val);
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
	
	ret = sensor_write(sd, 0x00, 0x00);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_vflip!\n");
		return ret;
	}
	
	ret = sensor_read(sd, 0x4, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_vflip!\n");
		return ret;
	}
	
	switch (value) {
		case 0:
		  val &= 0x00;
			break;
		case 1:
			val |= 0x02;
			break;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, 0x04, val);
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
	
	ret = sensor_write(sd, 0x00, 0x01);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_autoexp!\n");
		return ret;
	}
	
	ret = sensor_read(sd, 0x10, &val);
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
//	struct regval_list regs;
	unsigned char val;
	
	ret = sensor_write(sd, 0x00, 0x01);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_autoexp!\n");
		return ret;
	}
	
	ret = sensor_read(sd, 0x10, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_autoexp!\n");
		return ret;
	}
	switch (value) {
		case V4L2_EXPOSURE_AUTO:
		  val |= 0x80;
			break;
		case V4L2_EXPOSURE_MANUAL:
			val &= 0x80;//0x7f;
			break;
		case V4L2_EXPOSURE_SHUTTER_PRIORITY:
			return -EINVAL;    
		case V4L2_EXPOSURE_APERTURE_PRIORITY:
			return -EINVAL;
		default:
			return -EINVAL;
	}
	ret = sensor_write(sd, 0x10, val);
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
	
	ret = sensor_write(sd, 0x00, 0x02);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_g_autowb!\n");
		return ret;
	}
	
	ret = sensor_read(sd, 0x10, &val);
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
	
	ret = sensor_write(sd, 0x00, 0x02);
	if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_s_autowb!\n");
		return ret;
	}
	ret = sensor_read(sd, 0x10, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_s_autowb!\n");
		return ret;
			}

	switch(value) {
	case 0:
		val &= 0xd0;
		break;
	case 1:
		val |= 0x00;
		break;
	default:
		break;
	}	
	ret = sensor_write(sd, 0x10, val);
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
 	vfe_dev_dbg("sensor_s_flash_mode[0x%d]!\n",value);
//  struct vfe_dev *dev=(struct vfe_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
  #ifdef _FLASH_FUNC_
	config_flash_mode(sd, value, info->fl_dev_info);
  #endif
  info->flash_mode = value;
  return 0;
}

static int sensor_s_sw_stby(struct v4l2_subdev *sd, int on_off)
{
	int ret=0;
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
	return ret;
}
/*
 * Stuff that knows about the sensor.
 */
 
static int sensor_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
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
			//reset off io
		vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			mdelay(10);
		vfe_dev_print("disalbe oe!\n");
     // ret = sensor_write_array(sd, sensor_oe_disable_regs, ARRAY_SIZE(sensor_oe_disable_regs));
      if(ret < 0)
        vfe_dev_err("disalbe oe falied!\n");
      //software standby on
      ret = sensor_s_sw_stby(sd, CSI_STBY_ON);
      if(ret < 0)
        vfe_dev_err("soft stby falied!\n");
      //standby on io
      vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
      usleep_range(10000,12000);
      	mdelay(100);
			//inactive mclk after stadby in
      vfe_set_mclk(sd,OFF);
			//reset on io
			vfe_gpio_write(sd,RESET,CSI_RST_ON);
		 	mdelay(100);
			break;
		case CSI_SUBDEV_STBY_OFF:
     
			vfe_dev_dbg("CSI_SUBDEV_STBY_OFF\n");
			//active mclk before stadby out
      ret = sensor_s_sw_stby(sd, CSI_STBY_OFF);
      if(ret < 0)
        vfe_dev_err("soft stby off falied!\n");
      usleep_range(10000,12000);
			//standby off io
      vfe_gpio_write(sd,PWDN,CSI_STBY_OFF);
      	mdelay(100);
			//reset off io
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			mdelay(10);
			vfe_gpio_write(sd,RESET,CSI_RST_ON);
			mdelay(100);
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			mdelay(100);
			break;
		case CSI_SUBDEV_PWR_ON:
		vfe_dev_dbg("CSI_SUBDEV_PWR_ON\n");
			//power on reset
			vfe_gpio_set_status(sd,PWDN,1);//set the gpio to output
			vfe_gpio_set_status(sd,RESET,1);//set the gpio to output
			vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
			//reset on io
			vfe_gpio_write(sd,RESET,CSI_RST_ON);
			mdelay(10);
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
	
			//reset on io
			vfe_gpio_write(sd,PWDN,CSI_STBY_OFF);
			mdelay(10);
			//reset after power on
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);

			mdelay(10);
		vfe_gpio_write(sd,RESET,CSI_RST_ON);
			mdelay(100);
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			mdelay(100);
			break;
	case CSI_SUBDEV_PWR_OFF:
      vfe_dev_dbg("CSI_SUBDEV_PWR_OFF\n");
      //reset io
			vfe_gpio_write(sd,PWDN,CSI_STBY_ON);
			mdelay(100);
			vfe_gpio_write(sd,RESET,CSI_RST_ON);
			mdelay(100);
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
	struct vfe_dev *dev=(struct vfe_dev *)dev_get_drvdata(sd->v4l2_dev->dev);

	switch(val)
	{
		case CSI_SUBDEV_RST_OFF:
			vfe_dev_dbg("CSI_SUBDEV_RST_OFF\n");
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_ON:
			vfe_dev_dbg("CSI_SUBDEV_RST_ON\n");
			vfe_gpio_write(sd,RESET,CSI_RST_ON);
			mdelay(10);
			break;
		case CSI_SUBDEV_RST_PUL:
			vfe_dev_dbg("CSI_SUBDEV_RST_PUL\n");
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			mdelay(10);
			vfe_gpio_write(sd,RESET,CSI_RST_ON);
			mdelay(10);
			vfe_gpio_write(sd,RESET,CSI_RST_OFF);
			mdelay(100);
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
	
	ret = sensor_write(sd, 0x00, 0x00);
		if (ret < 0) {
		vfe_dev_err("sensor_write err at sensor_detect!\n");
			return ret;
		}
	
	ret = sensor_read(sd, 0x01, &val);
	if (ret < 0) {
		vfe_dev_err("sensor_read err at sensor_detect!\n");
		return ret;
	}

	if(val != 0x1b)
		return -ENODEV;

	
	return 0;
}
/*
static int sensor_release_detect(struct v4l2_subdev *sd)
{
	int ret;
	unsigned char val;
	
	//usleep_range(10000,12000);
	
	ret = sensor_read(sd, 0x02, val);
	if (ret < 0) {
		vfe_dev_err(">>>>>>>>>>>>>>>> sensor_read err at sensor_detect!\n");
		return ret;
	}
	vfe_dev_err("2,,,,,,,, 2 %x!\n", val);
	if (val==0x10)//siv121d
	{
		vfe_dev_dbg(">>>>>>>>>>>>>>>> read sensor release is siv121d\n");
		return 0;
	}
	else if(val==0x15) //siv121du
	{
		vfe_dev_dbg(">>>>>>>>>>>>>>>> read sensor release is siv121du\n");
		return 1;
	}
	else
	{
		vfe_dev_dbg(">>>>>>>>>>>>>>>> read sensor release faild\n");
		return -ENODEV;
	}
	return -1;
}
*/
static int sensor_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct sensor_info *info = to_state(sd);
#ifdef _FLASH_FUNC_
	struct vfe_dev *dev=(struct vfe_dev *)dev_get_drvdata(sd->v4l2_dev->dev);
#endif
	vfe_dev_dbg("sensor_init\n");
	/*Make sure it is a target sensor*/
	ret = sensor_detect(sd);
	if (ret) {
		vfe_dev_err("chip found is not an target chip.\n");
		return ret;
	}
//	sensor_write_array(sd, sensor_default_regs_24M , ARRAY_SIZE(sensor_default_regs_24M));
	sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
#ifdef _FLASH_FUNC_
	if(dev->flash_used==1) {
		info->fl_dev_info=&fl_info;
		info->fl_dev_info->dev_if=0;
		info->fl_dev_info->en_pol=FLASH_EN_POL;
		info->fl_dev_info->fl_mode_pol=FLASH_MODE_POL;
		info->fl_dev_info->light_src=0x01;
		info->fl_dev_info->flash_intensity=400;
		info->fl_dev_info->flash_level=0x01;
		info->fl_dev_info->torch_intensity=200;
		info->fl_dev_info->torch_level=0x01;
		info->fl_dev_info->timeout_counter=300*1000;
		config_flash_mode(sd, V4L2_FLASH_LED_MODE_NONE,
		                  info->fl_dev_info);
		io_set_flash_ctrl(sd, SW_CTRL_FLASH_OFF, info->fl_dev_info);
	}
#endif
	return 0;
	//return sensor_write_array(sd, siv121du_sensor_default_regs , ARRAY_SIZE(siv121du_sensor_default_regs));
	/*
	ret = sensor_release_detect(sd);
	if(ret == 0){
		vfe_dev_dbg("##################### %s:  read sensor release is siv121d\n", __func__);
		return sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
	}else	
	{ 
        vfe_dev_dbg("#####################  %s: read sensor release is siv121du\n", __func__);
		return sensor_write_array(sd, siv121du_sensor_default_regs , ARRAY_SIZE(siv121du_sensor_default_regs));
	}
*/
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
		.regs 			= sensor_vga_regs,
		.regs_size	= ARRAY_SIZE(sensor_vga_regs),
		.set_size		= NULL,
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
		.width			= SVGA_WIDTH,
		.height			= SVGA_HEIGHT,
    .hoffset    = 0,
    .voffset    = 0,
		.regs				= sensor_svga_regs,
		.regs_size	= ARRAY_SIZE(sensor_svga_regs),
		.set_size		= NULL,
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
		//struct v4l2_format *fmt,
		struct v4l2_mbus_framefmt *fmt,//linux-3.0
		struct sensor_format_struct **ret_fmt,
		struct sensor_win_size **ret_wsize)
{
	int index;
	struct sensor_win_size *wsize;
//	struct v4l2_pix_format *pix = &fmt->fmt.pix;//linux-3.0
	for (index = 0; index < N_FMTS; index++)
		if (sensor_formats[index].mbus_code == fmt->code)//linux-3.0
			break;
	
  if (index >= N_FMTS) 
    return -EINVAL;
  
	
	if (ret_fmt != NULL)
		*ret_fmt = sensor_formats + index;
		
	/*
	 * Fields: the sensor devices claim to be progressive.
	 */
	fmt->field = V4L2_FIELD_NONE;//linux-3.0
	
	
	/*
	 * Round requested image size down to the nearest
	 * we support, but not below the smallest.
	 */
	for (wsize = sensor_win_sizes; wsize < sensor_win_sizes + N_WIN_SIZES;
	     wsize++)
		if (fmt->width >= wsize->width && fmt->height >= wsize->height)//linux-3.0
			break;
	
	if (wsize >= sensor_win_sizes + N_WIN_SIZES)
		wsize--;   /* Take the smallest one */
	if (ret_wsize != NULL)
		*ret_wsize = wsize;
	/*
	 * Note the size we'll actually handle.
	 */
//#if 1 

	fmt->width = wsize->width;//linux-3.0
	fmt->height = wsize->height;//linux-3.0
	//pix->bytesperline = pix->width*sensor_formats[index].bpp;//linux-3.0
	//pix->sizeimage = pix->height*pix->bytesperline;//linux-3.0
	
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
 //zhg
static int sensor_s_fmt(struct v4l2_subdev *sd, 
             struct v4l2_mbus_framefmt *fmt)//linux-3.0
{
	int ret;
	struct sensor_format_struct *sensor_fmt;
	struct sensor_win_size *wsize;
	struct sensor_info *info = to_state(sd);
	vfe_dev_dbg("sensor_s_fmt\n");
	ret = sensor_try_fmt_internal(sd, fmt, &sensor_fmt, &wsize);
	if (ret){
		return ret;
		
	}
	if(info->capture_mode == V4L2_MODE_VIDEO) {
	//video
#ifdef _FLASH_FUNC_
		if(info->flash_mode!=V4L2_FLASH_LED_MODE_NONE) {
			//printk("shut flash when preview\n");
			io_set_flash_ctrl(sd, SW_CTRL_FLASH_OFF, info->fl_dev_info);
		}
#endif
	} else if(info->capture_mode == V4L2_MODE_IMAGE) {
#ifdef _FLASH_FUNC_
		check_to_flash(sd);
#endif

#ifdef _FLASH_FUNC_
		if(info->flash_mode!=V4L2_FLASH_LED_MODE_NONE) {
			if(to_flash==1) {
				vfe_dev_dbg("open flash when capture\n");
				io_set_flash_ctrl(sd, SW_CTRL_FLASH_ON, info->fl_dev_info);
				msleep(50);
			}
		}
#endif

		ret = sensor_s_autowb(sd,0); //lock wb
		if (ret < 0)
		  vfe_dev_err("sensor_s_autowb off err when capturing image!\n");
	}
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
//sensor_write(sd, REG_CLKRC, info->clkrc);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	struct sensor_info *info = to_state(sd);
	unsigned char div;
	
	vfe_dev_dbg("sensor_s_parm\n");	
	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE){
		vfe_dev_dbg("parms->type!=V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		return -EINVAL;
	}
	  
	info->capture_mode = cp->capturemode;
	if (info->capture_mode == V4L2_MODE_IMAGE) {
		vfe_dev_dbg("capture mode is not video mode,can not set frame rate!\n");
		return 0;
}
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
		return sensor_g_colorfx(sd,	&ctrl->value);
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
	  
	  //joe--20120223
//	case V4L2_CID_ZOOM_ABSOLUTE:
	//  return sensor_s_zoom(sd,
	 //     (int) ctrl->value);
	  
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
	.enum_mbus_fmt = sensor_enum_fmt,//linux-3.0
  .enum_framesizes = sensor_enum_size,
	.try_mbus_fmt = sensor_try_fmt,//linux-3.0
	.s_mbus_fmt = sensor_s_fmt,//linux-3.0
	.s_parm = sensor_s_parm,//linux-3.0
	.g_parm = sensor_g_parm,//linux-3.0
  .g_mbus_config = sensor_g_mbus_config,
};

static const struct v4l2_subdev_ops sensor_ops = {
	.core = &sensor_core_ops,
	.video = &sensor_video_ops,
};

/* ----------------------------------------------------------------------- */
static struct cci_driver cci_drv = {
	.name = "sid130b",
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
	//v4l2_i2c_subdev_init(sd, client, &sensor_ops);
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
//	info->clkrc = 1;	/* 30fps */

	return 0;
}


static int sensor_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd ;

	sd = cci_dev_remove_helper(client, &cci_drv);
	kfree(to_state(sd));
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "sid130b", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

//linux-3.0
static struct i2c_driver sensor_driver = {
	.driver = {
		.owner = THIS_MODULE,
	.name = "sid130b",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};
static __init int init_sensor(void)
{
	return cci_dev_init_helper(&sensor_driver);
}

static __exit void exit_sensor(void)
{
  cci_dev_exit_helper(&sensor_driver);
}

module_init(init_sensor);
module_exit(exit_sensor);
