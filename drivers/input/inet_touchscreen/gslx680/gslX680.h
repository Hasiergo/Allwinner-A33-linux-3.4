#ifndef _GSLX680_H_
#define _GSLX680_H_


struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};

//#define STRETCH_FRAME
#ifdef STRETCH_FRAME
#define CTP_MAX_X 		SCREEN_MAX_X
#define CTP_MAX_Y 		SCREEN_MAX_Y

#define X_STRETCH_MAX	(CTP_MAX_X/12)
#define Y_STRETCH_MAX	(CTP_MAX_Y/20)

#define XL_RATIO_1	40
#define XL_RATIO_2	65
#define XR_RATIO_1	20
#define XR_RATIO_2	30
#define YL_RATIO_1	25
#define YL_RATIO_2	35
#define YR_RATIO_1	25
#define YR_RATIO_2	35

#define X_STRETCH_CUST	(CTP_MAX_X/2)
#define Y_STRETCH_CUST	(CTP_MAX_Y/2)
#define X_RATIO_CUST	2
#define Y_RATIO_CUST	2
#endif

//#define FILTER_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	6
#endif



#endif

