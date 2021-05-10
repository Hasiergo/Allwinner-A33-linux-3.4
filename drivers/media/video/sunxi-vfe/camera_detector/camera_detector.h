#ifndef __CAMERA_DETECTOR_H__
#define __CAMERA_DETECTOR_H__

#include "camera_includes.h"

//error number
#define ECFGNOEX            1   //config not exit
#define ECFGERR             2   //config error
#define ECAMNUMERR          3   //camera number error
#define ECLKSRC             4   //clock source error
#define EDETECTFAIL         5   //detect camera fail
#define EPMUPIN             6   //get pmu pin fail
#define ENORMALPIN          7   //get normal pin fail
#define ECFGPIN             8   //config pin fail
#define EI2CADAPTER         9   //get I2C adapter fail

typedef struct {
    __u32                   num;
    __camera_csi_connect_e  csi_connect_type;
    __camera_info_t         camera[MAX_CAMERA_NUM];
}__camera_detector_t;

enum sensor_fmt {
  YUV,
  RAW_2M,
  RAW_5M
};


typedef struct __camera_sensor_info_{
	char * sensor_name;
	__u32 pixels;
	__u32 sensor_fmt;
	/*
	__u32 h_flip;	
	__u32 v_flip;
	*/
}__camera_sensor_info;

int  camera_gpio_set_range(struct gpio_config  *gpio, __u32 i);

#endif
