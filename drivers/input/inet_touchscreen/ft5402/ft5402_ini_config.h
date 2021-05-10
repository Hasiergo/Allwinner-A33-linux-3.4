#ifndef __LINUX_FT5402_INI_CONFIG_H__
#define __LINUX_FT5402_INI_CONFIG_H


/*Init param register address*/
/*factory mode register	from 14-131*/
#define FT5402_REG_TX_NUM								0x03//(14)
#define FT5402_REG_RX_NUM								0x04//(15)
#define FT5402_REG_VOLTAGE								0x05//(16)
#define FT5402_REG_GAIN								0x07//(18)
#define FT5402_REG_SCAN_SELECT						0x4E//(19)
#define FT5402_REG_TX_ORDER_START					0x50//(22-49)
#define FT5402_REG_TX_CAP_START						0x78//(66-93)
#define FT5402_REG_TX_OFFSET_START					0xBF//(110-123)
#define FT5402_REG_RX_ORDER_START					0xeb//(50-65)
#define FT5402_REG_RX_CAP_START						0xA0//(94-109)
#define FT5402_REG_RX_OFFSET_START					0xD3//(124-131)
#define FT5402_REG_START_RX							0x06//(17)
#define FT5402_REG_ADC_TARGET_HIGH					0x08//(20)
#define FT5402_REG_ADC_TARGET_LOW					0x09//(21)


#define FT5402_REG_DEVICE_MODE						0x00


/*work mode register	from 0-13(0,1,12,13verify or Reserved)and 132-177(159 Reserved)*/
#define FT5402_REG_THGROUP							(0x00+0x80)//(139-140)
#define FT5402_REG_THPEAK 								(0x01+0x80)//(141-142)
#define FT5402_REG_PWMODE_CTRL						(0x06+0x80)//(132)
#define FT5402_REG_PERIOD_ACTIVE						(0x59+0x80)//(162)
#define FT5402_REG_POINTS_SUPPORTED					(0x0A+0x80)//(133)
#define FT5402_REG_ESD_FILTER_FRAME					(0x4F+0x80)//(151)

#define FT5402_REG_RESOLUTION_X_H						(0x18+0x80)//(6)
#define FT5402_REG_RESOLUTION_X_L						(0x19+0x80)//(7)
#define FT5402_REG_RESOLUTION_Y_H						(0x1a+0x80)//(8)
#define FT5402_REG_RESOLUTION_Y_L						(0x1b+0x80)//(9)
#define FT5402_REG_KX_H								(0x1c+0x80)//(2)
#define FT5402_REG_KX_L								(0x9d)//(3)
#define FT5402_REG_KY_H								(0x9e)//(4)
#define FT5402_REG_KY_L								(0x1f+0x80)//(5)
#define FT5402_REG_CUSTOMER_ID						(0xA8) //(154)
#define FT5402_REG_DRAW_LINE_TH						(0xAe)//(134)
#define FT5402_REG_FACE_DETECT_MODE					(0xB0)//(135)
#define FT5402_REG_MAX_TOUCH_VALUE_HIGH				(0xD0)//(152)
#define FT5402_REG_MAX_TOUCH_VALUE_LOW				(0xD1)//(153)

#define FT5402_REG_DIRECTION							(0x53+0x80)//(156)
#define FT5402_REG_LEMDA_X							(0x41+0x80)//(10)
#define FT5402_REG_LEMDA_Y							(0x42+0x80)//(11)
#define FT5402_REG_FACE_DETECT_STATISTICS_TX_NUM	(0x43+0x80)//(136)
#define FT5402_REG_FACE_DETECT_PRE_VALUE			(0x44+0x80)//(137)
#define FT5402_REG_FACE_DETECT_NUM					(0x45+0x80)//(138)
#define FT5402_REG_BIGAREA_PEAK_VALUE_MIN			(0x33+0x80)//(143-144)
#define FT5402_REG_BIGAREA_DIFF_VALUE_OVER_NUM		(0x34+0x80)//(145)

/**************************************************************************/
#define FT5402_REG_FT5402_POINTS_STABLE_MACRO				(0x57+0x80)//(160)
#define FT5402_REG_FT5402_MIN_DELTA_X						(0x4a+0x80)//(146)
#define FT5402_REG_FT5402_MIN_DELTA_Y						(0x4b+0x80)//(147)
#define FT5402_REG_FT5402_MIN_DELTA_STEP						(0x4c+0x80)//(148)

#define FT5402_REG_FT5402_ESD_NOISE_MACRO					(0x58+0x80)//(161)
#define FT5402_REG_FT5402_ESD_DIFF_VAL						(0x4d+0x80)//(149)
#define FT5402_REG_FT5402_ESD_NEGTIVE						(0xCe)//(150)
#define FT5402_REG_FT5402_ESD_FILTER_FRAMES					(0x4f+0x80)//(151)

#define FT5402_REG_FT5402_IO_LEVEL_SELECT					(0x52+0x80)//(155)

#define FT5402_REG_FT5402_POINTID_DELAY_COUNT				(0x54+0x80)//(157)

#define FT5402_REG_FT5402_LIFTUP_FILTER_MACRO				(0x55+0x80)//(158)

#define FT5402_REG_FT5402_DIFF_HANDLE_MACRO					(0x5A+0x80)//(163)
#define FT5402_REG_FT5402_MIN_WATER							(0x5B+0x80)//(164)
#define FT5402_REG_FT5402_MAX_NOISE							(0x5C+0x80)//(165)
#define FT5402_REG_FT5402_WATER_START_RX					(0x5D+0x80)//(166)
#define FT5402_REG_FT5402_WATER_START_TX					(0xDE)//(167)

#define FT5402_REG_FT5402_HOST_NUMBER_SUPPORTED_MACRO	(0x38+0x80)//(168)
#define FT5402_REG_FT5402_RAISE_THGROUP						(0x36+0x80)//(169)
#define FT5402_REG_FT5402_CHARGER_STATE						(0x35+0x80)//(170)

#define FT5402_REG_FT5402_FILTERID_START						(0x37+0x80)//(171)

#define FT5402_REG_FT5402_FRAME_FILTER_EN_MACRO			(0x5F+0x80)//(172)
#define FT5402_REG_FT5402_FRAME_FILTER_SUB_MAX_TH			(0x60+0x80)//(173)
#define FT5402_REG_FT5402_FRAME_FILTER_ADD_MAX_TH			(0x61+0x80)//(174)
#define FT5402_REG_FT5402_FRAME_FILTER_SKIP_START_FRAME	(0x62+0x80)//(175)
#define FT5402_REG_FT5402_FRAME_FILTER_BAND_EN				(0x63+0x80)//(176)
#define FT5402_REG_FT5402_FRAME_FILTER_BAND_WIDTH			(0x64+0x80)//(177)

//mbg ++ 20131120
#define FT5402_REG_POWER_NOISE_FILTER_PROCESS_EN			(0x65+0x80)
#define FT5402_REG_POWER_NOISE_RX_PEAK_DIFF					(0x66+0x80)
#define FT5402_REG_POWER_NOISE_RX_NUM						(0x67+0x80)
//mbg --


/**************************************************************************/

#define FT5402_REG_TEST_MODE			0x04
#define FT5402_REG_TEST_MODE_2		0x05
#define FT5402_TX_TEST_MODE_1			0x28
#define FT5402_RX_TEST_MODE_1			0x1E
#define FT5402_FACTORYMODE_VALUE		0x40
#define FT5402_WORKMODE_VALUE		0x00


/************************************************************************/
/* string                                                               */
/************************************************************************/
#define STRING_FT5402_KX				"FT5402_KX"
#define STRING_FT5402_KY				"FT5402_KY"
#define STRING_FT5402_LEMDA_X			"FT5402_LEMDA_X"
#define STRING_FT5402_LEMDA_Y			"FT5402_LEMDA_Y"
#define STRING_FT5402_RESOLUTION_X	"FT5402_RESOLUTION_X"
#define STRING_FT5402_RESOLUTION_Y	"FT5402_RESOLUTION_Y"
#define STRING_FT5402_DIRECTION		"FT5402_DIRECTION"



#define STRING_FT5402_FACE_DETECT_PRE_VALUE			"FT5402_FACE_DETECT_PRE_VALUE"
#define STRING_FT5402_FACE_DETECT_NUM				"FT5402_FACE_DETECT_NUM"
#define STRING_FT5402_BIGAREA_PEAK_VALUE_MIN		"FT5402_BIGAREA_PEAK_VALUE_MIN"
#define STRING_FT5402_BIGAREA_DIFF_VALUE_OVER_NUM	"FT5402_BIGAREA_DIFF_VALUE_OVER_NUM"
#define STRING_FT5402_CUSTOMER_ID						"FT5402_CUSTOMER_ID"
#define STRING_FT5402_PERIOD_ACTIVE					"FT5402_PERIOD_ACTIVE"
#define STRING_FT5402_FACE_DETECT_STATISTICS_TX_NUM	"FT5402_FACE_DETECT_STATISTICS_TX_NUM"

#define STRING_FT5402_THGROUP							"FT5402_THGROUP"
#define STRING_FT5402_THPEAK							"FT5402_THPEAK"
#define STRING_FT5402_FACE_DETECT_MODE				"FT5402_FACE_DETECT_MODE"
#define STRING_FT5402_MAX_TOUCH_VALUE				"FT5402_MAX_TOUCH_VALUE"

#define STRING_FT5402_PWMODE_CTRL					"FT5402_PWMODE_CTRL"
#define STRING_FT5402_DRAW_LINE_TH					"FT5402_DRAW_LINE_TH"

#define STRING_FT5402_POINTS_SUPPORTED				"FT5402_POINTS_SUPPORTED"

#define STRING_FT5402_START_RX						"FT5402_START_RX"
#define STRING_FT5402_ADC_TARGET						"FT5402_ADC_TARGET"
#define STRING_FT5402_ESD_FILTER_FRAME				"FT5402_ESD_FILTER_FRAME"

#define STRING_FT5402_POINTS_STABLE_MACRO			"FT5402_POINTS_STABLE_MACRO"
#define STRING_FT5402_MIN_DELTA_X						"FT5402_MIN_DELTA_X"
#define STRING_FT5402_MIN_DELTA_Y						"FT5402_MIN_DELTA_Y"
#define STRING_FT5402_MIN_DELTA_STEP					"FT5402_MIN_DELTA_STEP	"

#define STRING_FT5402_ESD_NOISE_MACRO				"FT5402_ESD_NOISE_MACRO"
#define STRING_FT5402_ESD_DIFF_VAL					"FT5402_ESD_DIFF_VAL"
#define STRING_FT5402_ESD_NEGTIVE						"FT5402_ESD_NEGTIVE"
#define STRING_FT5402_ESD_FILTER_FRAME				"FT5402_ESD_FILTER_FRAME"

#define STRING_FT5402_IO_LEVEL_SELECT					"FT5402_IO_LEVEL_SELECT"
#define STRING_FT5402_POINTID_DELAY_COUNT			"FT5402_POINTID_DELAY_COUNT"

#define STRING_FT5402_LIFTUP_FILTER_MACRO			"FT5402_LIFTUP_FILTER_MACRO"

#define STRING_FT5402_DIFFDATA_HANDLE				"FT5402_DIFFDATA_HANDLE	"	//_MACRO
#define STRING_FT5402_MIN_WATER_VAL					"FT5402_MIN_WATER_VAL"
#define STRING_FT5402_MAX_NOISE_VAL					"FT5402_MAX_NOISE_VAL"
#define STRING_FT5402_WATER_HANDLE_START_RX		"FT5402_WATER_HANDLE_START_RX"
#define STRING_FT5402_WATER_HANDLE_START_TX		"FT5402_WATER_HANDLE_START_TX"

#define STRING_FT5402_HOST_NUMBER_SUPPORTED		"FT5402_HOST_NUMBER_SUPPORTED"
#define STRING_FT5402_RV_G_RAISE_THGROUP			"FT5402_RV_G_RAISE_THGROUP"
#define STRING_FT5402_RV_G_CHARGER_STATE			"FT5402_RV_G_CHARGER_STATE"

#define STRING_FT5402_RV_G_FILTERID_START			"FT5402_RV_G_FILTERID_START"

#define STRING_FT5402_FRAME_FILTER_EN				"FT5402_FRAME_FILTER_EN"
#define STRING_FT5402_FRAME_FILTER_SUB_MAX_TH		"FT5402_FRAME_FILTER_SUB_MAX_TH"
#define STRING_FT5402_FRAME_FILTER_ADD_MAX_TH		"FT5402_FRAME_FILTER_ADD_MAX_TH"
#define STRING_FT5402_FRAME_FILTER_SKIP_START_FRAME	"FT5402_FRAME_FILTER_SKIP_START_FRAME"
#define STRING_FT5402_FRAME_FILTER_BAND_EN			"FT5402_FRAME_FILTER_BAND_EN"
#define STRING_FT5402_FRAME_FILTER_BAND_WIDTH		"FT5402_FRAME_FILTER_BAND_WIDTH"

//mbg ++ 20131120
#define STRING_FT5402_POWER_NOISE_FILTER_PROCESS_EN		"FT5402_POWER_NOISE_FILTER_PROCESS_EN"
#define STRING_FT5402_POWER_NOISE_RX_PEAK_DIFF				"FT5402_POWER_NOISE_RX_PEAK_DIFF"
#define STRING_FT5402_POWER_NOISE_RX_NUM					"FT5402_POWER_NOISE_RX_NUM"

//mbg --


#define STRING_ft5402_tx_num	"FT5402_tx_num"
#define STRING_ft5402_rx_num	"FT5402_rx_num"
#define STRING_ft5402_gain		"FT5402_gain"
#define STRING_ft5402_voltage	"FT5402_voltage"
#define STRING_ft5402_scanselect "FT5402_scanselect"

#define STRING_ft5402_tx_order 	"FT5402_tx_order"
#define STRING_ft5402_tx_offset	"FT5402_tx_offset"
#define STRING_ft5402_tx_cap 	"FT5402_tx_cap"

#define STRING_ft5402_rx_order 	"FT5402_rx_order"
#define STRING_ft5402_rx_offset 	"FT5402_rx_offset"
#define STRING_ft5402_rx_cap 	"FT5402_rx_cap"


struct Struct_Param_FT5402 {
	short ft5402_KX;
	short ft5402_KY;
	unsigned char ft5402_LEMDA_X;
	unsigned char ft5402_LEMDA_Y;
	short ft5402_RESOLUTION_X;
	short ft5402_RESOLUTION_Y;
	unsigned char ft5402_DIRECTION;
	unsigned char ft5402_FACE_DETECT_PRE_VALUE;
	unsigned char ft5402_FACE_DETECT_NUM;

	unsigned char ft5402_BIGAREA_PEAK_VALUE_MIN;
	unsigned char ft5402_BIGAREA_DIFF_VALUE_OVER_NUM;
	unsigned char ft5402_CUSTOMER_ID;
	unsigned char ft5402_PERIOD_ACTIVE;
	unsigned char  ft5402_FACE_DETECT_STATISTICS_TX_NUM;

	//short ft5402_THGROUP;
	unsigned char ft5402_THGROUP;
	unsigned char ft5402_THPEAK;
	unsigned char ft5402_FACE_DETECT_MODE;
	short ft5402_MAX_TOUCH_VALUE;

	unsigned char ft5402_PWMODE_CTRL;
	unsigned char ft5402_DRAW_LINE_TH;
	unsigned char ft5402_POINTS_SUPPORTED;
		
	unsigned char ft5402_START_RX;
	short ft5402_ADC_TARGET;
	unsigned char ft5402_ESD_FILTER_FRAME;

	unsigned char ft5402_POINTS_STABLE_MACRO;
	unsigned char ft5402_MIN_DELTA_X;
	unsigned char ft5402_MIN_DELTA_Y;
	unsigned char ft5402_MIN_DELTA_STEP;
	
	unsigned char ft5402_ESD_NOISE_MACRO;
	unsigned char ft5402_ESD_DIFF_VAL;
	char ft5402_ESD_NEGTIVE;	//negtive
	unsigned char ft5402_ESD_FILTER_FRAMES;

	unsigned char ft5402_IO_LEVEL_SELECT;

	unsigned char ft5402_POINTID_DELAY_COUNT;

	unsigned char ft5402_LIFTUP_FILTER_MACRO;

	unsigned char ft5402_DIFF_HANDLE_MACRO;
	char ft5402_MIN_WATER;	//negtive
	unsigned char ft5402_MAX_NOISE;
	unsigned char ft5402_WATER_START_RX;
	unsigned char ft5402_WATER_START_TX;

	unsigned char ft5402_HOST_NUMBER_SUPPORTED_MACRO;
	unsigned char ft5402_RAISE_THGROUP;
	unsigned char ft5402_CHARGER_STATE;

	unsigned char ft5402_FILTERID_START;

	unsigned char ft5402_FRAME_FILTER_EN_MACRO;
	unsigned char ft5402_FRAME_FILTER_SUB_MAX_TH;
	unsigned char ft5402_FRAME_FILTER_ADD_MAX_TH;
	unsigned char ft5402_FRAME_FILTER_SKIP_START_FRAME;
	unsigned char ft5402_FRAME_FILTER_BAND_EN;
	unsigned char ft5402_FRAME_FILTER_BAND_WIDTH;

	//mbg ++ 20131120
	unsigned char ft5402_POWER_NOISE_FILTER_PROCESS_EN;
	unsigned char ft5402_POWER_NOISE_RX_PEAK_DIFF;
	unsigned char ft5402_POWER_NOISE_RX_NUM;	
	//mbg --
	
};

struct Struct_Param_FT5402 g_param_ft5402;
/*
struct Struct_Param_FT5402 g_param_ft5402 = {
	FT5402_KX,
	FT5402_KY,
	FT5402_LEMDA_X,
	FT5402_LEMDA_Y,
	FT5402_RESOLUTION_X,
	FT5402_RESOLUTION_Y,
	FT5402_DIRECTION,

	FT5402_FACE_DETECT_PRE_VALUE,
	FT5402_FACE_DETECT_NUM,
	FT5402_BIGAREA_PEAK_VALUE_MIN,
	FT5402_BIGAREA_DIFF_VALUE_OVER_NUM,
	FT5402_CUSTOMER_ID,
	FT5402_RV_G_PERIOD_ACTIVE,	//FT5402_PERIOD_ACTIVE,
	FT5402_FACE_DETECT_STATISTICS_TX_NUM,

	FT5402_THGROUP,
	FT5402_THPEAK,
	FT5402_FACE_DETECT_MODE,
	FT5402_MAX_TOUCH_VALUE,

	FT5402_PWMODE_CTRL,
	FT5402_DRAW_LINE_TH,
	FT5402_POINTS_SUPPORTED,

	FT5402_START_RX,
	FT5402_ADC_TARGET,
	FT5402_ESD_FILTER_FRAME,

	FT5402_POINTS_STABLE_MACRO,
	FT5402_MIN_DELTA_X,
	FT5402_MIN_DELTA_Y,
	FT5402_MIN_DELTA_STEP,
	
	FT5402_ESD_NOISE_MACRO,
	FT5402_ESD_DIFF_VAL,
	FT5402_ESD_NEGTIVE,
	FT5402_ESD_FILTER_FRAME,

	FT5402_IO_LEVEL_SELECT,

	FT5402_POINTID_DELAY_COUNT,

	FT5402_LIFTUP_FILTER_MACRO,

	FT5402_DIFFDATA_HANDLE,//_MACRO
	FT5402_MIN_WATER_VAL,
	FT5402_MAX_NOISE_VAL,
	FT5402_WATER_HANDLE_START_RX,
	FT5402_WATER_HANDLE_START_TX,

	FT5402_HOST_NUMBER_SUPPORTED,//_MACRO
	FT5402_RV_G_RAISE_THGROUP,
	FT5402_RV_G_CHARGER_STATE,

	FT5402_RV_G_FILTERID_START,

	FT5402_FRAME_FILTER_EN,//_MACRO
	FT5402_FRAME_FILTER_SUB_MAX_TH,
	FT5402_FRAME_FILTER_ADD_MAX_TH,
	FT5402_FRAME_FILTER_SKIP_START_FRAME,
	FT5402_FRAME_FILTER_BAND_EN,
	FT5402_FRAME_FILTER_BAND_WIDTH,	

	//mbg ++ 20131120
	FT5402_POWER_NOISE_FILTER_PROCESS_EN,
	FT5402_POWER_NOISE_RX_PEAK_DIFF,
	FT5402_POWER_NOISE_RX_NUM,
	//mbg --
};
*/
char String_Param_FT5402[][64] = {
	STRING_FT5402_KX,
	STRING_FT5402_KY,
	STRING_FT5402_LEMDA_X,
	STRING_FT5402_LEMDA_Y,
	STRING_FT5402_RESOLUTION_X,
	STRING_FT5402_RESOLUTION_Y,
	STRING_FT5402_DIRECTION,
	STRING_FT5402_FACE_DETECT_PRE_VALUE,
	STRING_FT5402_FACE_DETECT_NUM,
	STRING_FT5402_BIGAREA_PEAK_VALUE_MIN,
	STRING_FT5402_BIGAREA_DIFF_VALUE_OVER_NUM,
	STRING_FT5402_CUSTOMER_ID,
	STRING_FT5402_PERIOD_ACTIVE,
	STRING_FT5402_FACE_DETECT_STATISTICS_TX_NUM,

	STRING_FT5402_THGROUP,
	STRING_FT5402_THPEAK,
	STRING_FT5402_FACE_DETECT_MODE,
	STRING_FT5402_MAX_TOUCH_VALUE,

	STRING_FT5402_PWMODE_CTRL,
	STRING_FT5402_DRAW_LINE_TH,
	STRING_FT5402_POINTS_SUPPORTED,
	
	STRING_FT5402_START_RX,
	STRING_FT5402_ADC_TARGET,
	STRING_FT5402_ESD_FILTER_FRAME,

	STRING_FT5402_POINTS_STABLE_MACRO,
	STRING_FT5402_MIN_DELTA_X,
	STRING_FT5402_MIN_DELTA_Y,
	STRING_FT5402_MIN_DELTA_STEP,
	
	STRING_FT5402_ESD_NOISE_MACRO,
	STRING_FT5402_ESD_DIFF_VAL,
	STRING_FT5402_ESD_NEGTIVE,
	STRING_FT5402_ESD_FILTER_FRAME,

	STRING_FT5402_IO_LEVEL_SELECT,

	STRING_FT5402_POINTID_DELAY_COUNT,

	STRING_FT5402_LIFTUP_FILTER_MACRO,

	STRING_FT5402_DIFFDATA_HANDLE,//_MACRO
	STRING_FT5402_MIN_WATER_VAL,
	STRING_FT5402_MAX_NOISE_VAL,
	STRING_FT5402_WATER_HANDLE_START_RX,
	STRING_FT5402_WATER_HANDLE_START_TX,

	STRING_FT5402_HOST_NUMBER_SUPPORTED,//_MACRO,
	STRING_FT5402_RV_G_RAISE_THGROUP,
	STRING_FT5402_RV_G_CHARGER_STATE,

	STRING_FT5402_RV_G_FILTERID_START,

	STRING_FT5402_FRAME_FILTER_EN,//_MACRO
	STRING_FT5402_FRAME_FILTER_SUB_MAX_TH,
	STRING_FT5402_FRAME_FILTER_ADD_MAX_TH,
	STRING_FT5402_FRAME_FILTER_SKIP_START_FRAME,
	STRING_FT5402_FRAME_FILTER_BAND_EN,
	STRING_FT5402_FRAME_FILTER_BAND_WIDTH,

	//mbg ++ 20131120
	STRING_FT5402_POWER_NOISE_FILTER_PROCESS_EN,
	STRING_FT5402_POWER_NOISE_RX_PEAK_DIFF,
	STRING_FT5402_POWER_NOISE_RX_NUM,
	//mbg --
	

	STRING_ft5402_tx_num,
	STRING_ft5402_rx_num,
	STRING_ft5402_gain,
	STRING_ft5402_voltage ,
	STRING_ft5402_scanselect,

	STRING_ft5402_tx_order,
	STRING_ft5402_tx_offset,
	STRING_ft5402_tx_cap,

	STRING_ft5402_rx_order,
	STRING_ft5402_rx_offset,
	STRING_ft5402_rx_cap,
};

#define FT5402_APP_NAME "FT5402_param"

#define FT5402_APP_LEGAL "Legal_File"
#define FT5402_APP_LEGAL_BYTE_1_STR "BYTE_1"
#define FT5402_APP_LEGAL_BYTE_2_STR "BYTE_2"

#define FT5402_APP_LEGAL_BYTE_1_VALUE 107
#define FT5402_APP_LEGAL_BYTE_2_VALUE 201


#define FT5402_INI_FILEPATH "/sdcard/"

#endif
