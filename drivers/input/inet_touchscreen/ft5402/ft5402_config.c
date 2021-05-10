#include "ft5402_config.h"
//#include <linux/i2c/ft5402_ts.h>

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <mach/irqs.h>

#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>

#include "ft5402_ini.h"
#include "ft5402_ini_config.h"
#include <mach/sys_config.h>
static int ctp_ft5402_num =0;


#define DBG printk
//extern int ft5402_i2c_Read(struct i2c_client *client,  char * writebuf, int writelen, char *readbuf, int readlen);
//extern int ft5402_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);
extern int ft5402_write_reg(struct i2c_client * client, u8 regaddr, u8 regvalue);
extern int ft5402_read_reg(struct i2c_client * client, u8 regaddr, u8 * regvalue);
/*set tx order
*@txNO:		offset from tx order start
*@txNO1:	tx NO.
*/
static int ft5402_set_tx_order(struct i2c_client * client, u8 txNO, u8 txNO1)
{
	unsigned char ReCode = 0;
	if (txNO < FT5402_TX_TEST_MODE_1)
		ReCode = ft5402_write_reg(client, FT5402_REG_TX_ORDER_START + txNO,
						txNO1);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if (ReCode >= 0)
			ReCode = ft5402_write_reg(client,
					FT5402_REG_TX_ORDER_START + txNO - FT5402_TX_TEST_MODE_1,
					txNO1);
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	return ReCode;
}

/*set tx order
*@txNO:		offset from tx order start
*@pTxNo:	return value of tx NO.
*/
static int ft5402_get_tx_order(struct i2c_client * client, u8 txNO, u8 *pTxNo)
{
	unsigned char ReCode = 0;
	if (txNO < FT5402_TX_TEST_MODE_1)
		ReCode = ft5402_read_reg(client, FT5402_REG_TX_ORDER_START + txNO,
						pTxNo);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if(ReCode >= 0)
			ReCode =  ft5402_read_reg(client,
					FT5402_REG_TX_ORDER_START + txNO - FT5402_TX_TEST_MODE_1,
					pTxNo);	
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	return ReCode;
}

/*set tx cap
*@txNO: 	tx NO.
*@cap_value:	value of cap
*/
static int ft5402_set_tx_cap(struct i2c_client * client, u8 txNO, u8 cap_value)
{
	unsigned char ReCode = 0;
	if (txNO < FT5402_TX_TEST_MODE_1)
		ReCode = ft5402_write_reg(client, FT5402_REG_TX_CAP_START + txNO,
						cap_value);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if (ReCode >= 0)
			ReCode = ft5402_write_reg(client,
					FT5402_REG_TX_CAP_START + txNO - FT5402_TX_TEST_MODE_1,
					cap_value);
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	return ReCode;
}

/*get tx cap*/
static int ft5402_get_tx_cap(struct i2c_client * client, u8 txNO, u8 *pCap)
{
	unsigned char ReCode = 0;
	if (txNO < FT5402_TX_TEST_MODE_1)
		ReCode =  ft5402_read_reg(client, FT5402_REG_TX_CAP_START + txNO,
					pCap);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if (ReCode >= 0)
			ReCode = ft5402_read_reg(client,
					FT5402_REG_TX_CAP_START + txNO - FT5402_TX_TEST_MODE_1,
					pCap);
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	return ReCode;
}

/*set tx offset*/
static int ft5402_set_tx_offset(struct i2c_client * client, u8 txNO, u8 offset_value)
{
	unsigned char temp=0;
	unsigned char ReCode = 0;
	if (txNO < FT5402_TX_TEST_MODE_1) {
		ReCode = ft5402_read_reg(client,
				FT5402_REG_TX_OFFSET_START + (txNO>>1), &temp);
		if (ReCode >= 0) {
			if (txNO%2 == 0)
				ReCode = ft5402_write_reg(client,
							FT5402_REG_TX_OFFSET_START + (txNO>>1),
							(temp&0xf0) + (offset_value&0x0f));	
			else
				ReCode = ft5402_write_reg(client,
							FT5402_REG_TX_OFFSET_START + (txNO>>1),
							(temp&0x0f) + (offset_value<<4));	
		}
	} else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if (ReCode >= 0) {
			ReCode = ft5402_read_reg(client,
				FT5402_REG_DEVICE_MODE+((txNO-FT5402_TX_TEST_MODE_1)>>1),
				&temp);	/*enter Test mode 2*/
			if (ReCode >= 0) {
				if(txNO%2 == 0)
					ReCode = ft5402_write_reg(client,
						FT5402_REG_TX_OFFSET_START+((txNO-FT5402_TX_TEST_MODE_1)>>1),
						(temp&0xf0)+(offset_value&0x0f));	
				else
					ReCode = ft5402_write_reg(client,
						FT5402_REG_TX_OFFSET_START+((txNO-FT5402_TX_TEST_MODE_1)>>1),
						(temp&0xf0)+(offset_value<<4));	
			}
		}
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	
	return ReCode;
}

/*get tx offset*/
static int ft5402_get_tx_offset(struct i2c_client * client, u8 txNO, u8 *pOffset)
{
	unsigned char temp=0;
	unsigned char ReCode = 0;
	if (txNO < FT5402_TX_TEST_MODE_1)
		ReCode = ft5402_read_reg(client,
				FT5402_REG_TX_OFFSET_START + (txNO>>1), &temp);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if (ReCode >= 0)
			ReCode = ft5402_read_reg(client,
						FT5402_REG_TX_OFFSET_START+((txNO-FT5402_TX_TEST_MODE_1)>>1),
						&temp);
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}

	if (ReCode >= 0)
		(txNO%2 == 0) ? (*pOffset = (temp&0x0f)) : (*pOffset = (temp>>4));
	return ReCode;
}

/*set rx order*/
static int ft5402_set_rx_order(struct i2c_client * client, u8 rxNO, u8 rxNO1)
{
	unsigned char ReCode = 0;
	ReCode = ft5402_write_reg(client, FT5402_REG_RX_ORDER_START + rxNO,
						rxNO1);
	return ReCode;
}

/*get rx order*/
static int ft5402_get_rx_order(struct i2c_client * client, u8 rxNO, u8 *prxNO1)
{
	unsigned char ReCode = 0;
	ReCode = ft5402_read_reg(client, FT5402_REG_RX_ORDER_START + rxNO,
						prxNO1);
	return ReCode;
}

/*set rx cap*/
static int ft5402_set_rx_cap(struct i2c_client * client, u8 rxNO, u8 cap_value)
{
	unsigned char ReCode = 0;
	if (rxNO < FT5402_RX_TEST_MODE_1)
		ReCode = ft5402_write_reg(client, FT5402_REG_RX_CAP_START + rxNO,
						cap_value);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if(ReCode >= 0)
			ReCode = ft5402_write_reg(client,
					FT5402_REG_RX_CAP_START + rxNO - FT5402_RX_TEST_MODE_1,
					cap_value);
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	
	return ReCode;
}

/*get rx cap*/
static int ft5402_get_rx_cap(struct i2c_client * client, u8 rxNO, u8 *pCap)
{
	unsigned char ReCode = 0;
	if (rxNO < FT5402_RX_TEST_MODE_1)
		ReCode = ft5402_read_reg(client, FT5402_REG_RX_CAP_START + rxNO,
						pCap);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if(ReCode >= 0)
			ReCode = ft5402_read_reg(client,
					FT5402_REG_RX_CAP_START + rxNO - FT5402_RX_TEST_MODE_1,
					pCap);
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	
	return ReCode;
}

/*set rx offset*/
static int ft5402_set_rx_offset(struct i2c_client * client, u8 rxNO, u8 offset_value)
{
	unsigned char temp=0;
	unsigned char ReCode = 0;
	if (rxNO < FT5402_RX_TEST_MODE_1) {
		ReCode = ft5402_read_reg(client,
				FT5402_REG_RX_OFFSET_START + (rxNO>>1), &temp);
		if (ReCode >= 0) {
			if (rxNO%2 == 0)
				ReCode = ft5402_write_reg(client,
							FT5402_REG_RX_OFFSET_START + (rxNO>>1),
							(temp&0xf0) + (offset_value&0x0f));	
			else
				ReCode = ft5402_write_reg(client,
							FT5402_REG_RX_OFFSET_START + (rxNO>>1),
							(temp&0x0f) + (offset_value<<4));	
		}
	}
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if (ReCode >= 0) {
			ReCode = ft5402_read_reg(client,
				FT5402_REG_DEVICE_MODE+((rxNO-FT5402_RX_TEST_MODE_1)>>1),
				&temp);	/*enter Test mode 2*/
			if (ReCode >= 0) {
				if (rxNO%2 == 0)
					ReCode = ft5402_write_reg(client,
						FT5402_REG_RX_OFFSET_START+((rxNO-FT5402_RX_TEST_MODE_1)>>1),
						(temp&0xf0)+(offset_value&0x0f));	
				else
					ReCode = ft5402_write_reg(client,
						FT5402_REG_RX_OFFSET_START+((rxNO-FT5402_RX_TEST_MODE_1)>>1),
						(temp&0xf0)+(offset_value<<4));	
			}
		}
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}
	
	return ReCode;
}

/*get rx offset*/
static int ft5402_get_rx_offset(struct i2c_client * client, u8 rxNO, u8 *pOffset)
{
	unsigned char temp = 0;
	unsigned char ReCode = 0;
	if (rxNO < FT5402_RX_TEST_MODE_1)
		ReCode = ft5402_read_reg(client,
				FT5402_REG_RX_OFFSET_START + (rxNO>>1), &temp);
	else {
		ReCode = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
						FT5402_REG_TEST_MODE_2<<4);	/*enter Test mode 2*/
		if (ReCode >= 0)
			ReCode = ft5402_read_reg(client,
						FT5402_REG_RX_OFFSET_START+((rxNO-FT5402_RX_TEST_MODE_1)>>1),
						&temp);
		
		ft5402_write_reg(client, FT5402_REG_DEVICE_MODE,
			FT5402_REG_TEST_MODE<<4);	/*enter Test mode*/
	}

	if (ReCode >= 0) {
		if (0 == (rxNO%2))
			*pOffset = (temp&0x0f);
		else
			*pOffset = (temp>>4);
	}
	
	return ReCode;
}

/*set tx num*/
static int ft5402_set_tx_num(struct i2c_client *client, u8 txnum)
{
	return ft5402_write_reg(client, FT5402_REG_TX_NUM, txnum);
}

/*get tx num*/
static int ft5402_get_tx_num(struct i2c_client *client, u8 *ptxnum)
{
	return ft5402_read_reg(client, FT5402_REG_TX_NUM, ptxnum);
}

/*set rx num*/
static int ft5402_set_rx_num(struct i2c_client *client, u8 rxnum)
{
	return ft5402_write_reg(client, FT5402_REG_RX_NUM, rxnum);
}

/*get rx num*/
static int ft5402_get_rx_num(struct i2c_client *client, u8 *prxnum)
{
	return ft5402_read_reg(client, FT5402_REG_RX_NUM, prxnum);
}

/*set resolution*/
static int ft5402_set_Resolution(struct i2c_client *client, u16 x, u16 y)
{
	unsigned char cRet = 0;
	cRet &= ft5402_write_reg(client,
			FT5402_REG_RESOLUTION_X_H, ((unsigned char)(x>>8)));
	cRet &= ft5402_write_reg(client,
			FT5402_REG_RESOLUTION_X_L, ((unsigned char)(x&0x00ff)));

	cRet &= ft5402_write_reg(client,
			FT5402_REG_RESOLUTION_Y_H, ((unsigned char)(y>>8)));
	cRet &= ft5402_write_reg(client,
			FT5402_REG_RESOLUTION_Y_L, ((unsigned char)(y&0x00ff)));

	return cRet;
}

/*get resolution*/
static int ft5402_get_Resolution(struct i2c_client *client,
			u16 *px, u16 *py)
{
	unsigned char cRet = 0, temp1 = 0, temp2 = 0;
	cRet &= ft5402_read_reg(client,
			FT5402_REG_RESOLUTION_X_H, &temp1);
	cRet &= ft5402_read_reg(client,
			FT5402_REG_RESOLUTION_X_L, &temp2);
	(*px) = (((u16)temp1) << 8) | ((u16)temp2);

	cRet &= ft5402_read_reg(client,
			FT5402_REG_RESOLUTION_Y_H, &temp1);
	cRet &= ft5402_read_reg(client,
			FT5402_REG_RESOLUTION_Y_L, &temp2);
	(*py) = (((u16)temp1) << 8) | ((u16)temp2);

	return cRet;
}


/*set voltage*/
static int ft5402_set_vol(struct i2c_client *client, u8 Vol)
{
	return  ft5402_write_reg(client, FT5402_REG_VOLTAGE, Vol);
}

/*get voltage*/
static int ft5402_get_vol(struct i2c_client *client, u8 *pVol)
{
	return ft5402_read_reg(client, FT5402_REG_VOLTAGE, pVol);
}

/*set gain*/
static int ft5402_set_gain(struct i2c_client *client, u8 Gain)
{
	return ft5402_write_reg(client, FT5402_REG_GAIN, Gain);
}

/*get gain*/
static int ft5402_get_gain(struct i2c_client *client, u8 *pGain)
{
	return ft5402_read_reg(client, FT5402_REG_GAIN, pGain);
}

/*get start rx*/
static int ft5402_get_start_rx(struct i2c_client *client, u8 *pRx)
{
	return ft5402_read_reg(client, FT5402_REG_START_RX, pRx);
}


/*get adc target*/
static int ft5402_get_adc_target(struct i2c_client *client, u16 *pvalue)
{
	int err = 0;
	u8 tmp1, tmp2;
	err = ft5402_read_reg(client, FT5402_REG_ADC_TARGET_HIGH,
			&tmp1);
	if (err < 0)
		dev_err(&client->dev, "%s:get adc target  high failed\n",
				__func__);
	err = ft5402_read_reg(client, FT5402_REG_ADC_TARGET_LOW,
			&tmp2);
	if (err < 0)
		dev_err(&client->dev, "%s:get adc target low failed\n",
				__func__);

	*pvalue = ((u16)tmp1<<8) + (u16)tmp2;
	return err;
}

static int ft5402_set_face_detect_statistics_tx_num(struct i2c_client *client, u8 prevalue)
{
	return ft5402_write_reg(client, FT5402_REG_FACE_DETECT_STATISTICS_TX_NUM,
			prevalue);
}

static int ft5402_get_face_detect_statistics_tx_num(struct i2c_client *client, u8 *pprevalue)
{
	return ft5402_read_reg(client, FT5402_REG_FACE_DETECT_STATISTICS_TX_NUM,
			pprevalue);
}

static int ft5402_set_face_detect_pre_value(struct i2c_client *client, u8 prevalue)
{
	return ft5402_write_reg(client, FT5402_REG_FACE_DETECT_PRE_VALUE,
			prevalue);
}

static int ft5402_get_face_detect_pre_value(struct i2c_client *client, u8 *pprevalue)
{
	return ft5402_read_reg(client, FT5402_REG_FACE_DETECT_PRE_VALUE,
			pprevalue);
}

static int ft5402_set_face_detect_num(struct i2c_client *client, u8 num)
{
	return ft5402_write_reg(client, FT5402_REG_FACE_DETECT_NUM,
			num);
}

static int ft5402_get_face_detect_num(struct i2c_client *client, u8 *pnum)
{
	return ft5402_read_reg(client, FT5402_REG_FACE_DETECT_NUM,
			pnum);
}


static int ft5402_set_peak_value_min(struct i2c_client *client, u8 min)
{
	return ft5402_write_reg(client, FT5402_REG_BIGAREA_PEAK_VALUE_MIN,
			min);
}

static int ft5402_get_peak_value_min(struct i2c_client *client, u8 *pmin)
{
	return ft5402_read_reg(client, FT5402_REG_BIGAREA_PEAK_VALUE_MIN,
			pmin);
}

static int ft5402_set_diff_value_over_num(struct i2c_client *client, u8 num)
{
	return ft5402_write_reg(client, FT5402_REG_BIGAREA_DIFF_VALUE_OVER_NUM,
			num);
}
static int ft5402_get_diff_value_over_num(struct i2c_client *client, u8 *pnum)
{
	return ft5402_read_reg(client, FT5402_REG_BIGAREA_DIFF_VALUE_OVER_NUM,
			pnum);
}


static int ft5402_set_customer_id(struct i2c_client *client, u8 num)
{
	return ft5402_write_reg(client, FT5402_REG_CUSTOMER_ID,
			num);
}
static int ft5402_get_customer_id(struct i2c_client *client, u8 *pnum)
{
	return ft5402_read_reg(client, FT5402_REG_CUSTOMER_ID,
			pnum);
}

static int ft5402_set_kx(struct i2c_client *client, u16 value)
{
	int err = 0;
	err = ft5402_write_reg(client, FT5402_REG_KX_H,
			value >> 8);
	if (err < 0)
		dev_err(&client->dev, "%s:set kx high failed\n",
				__func__);
	err = ft5402_write_reg(client, FT5402_REG_KX_L,
			value);
	if (err < 0)
		dev_err(&client->dev, "%s:set kx low failed\n",
				__func__);

	return err;
}

static int ft5402_get_kx(struct i2c_client *client, u16 *pvalue)
{
	int err = 0;
	u8 tmp1, tmp2;
	err = ft5402_read_reg(client, FT5402_REG_KX_H,
			&tmp1);
	if (err < 0)
		dev_err(&client->dev, "%s:get kx high failed\n",
				__func__);
	err = ft5402_read_reg(client, FT5402_REG_KX_L,
			&tmp2);
	if (err < 0)
		dev_err(&client->dev, "%s:get kx low failed\n",
				__func__);

	*pvalue = ((u16)tmp1<<8) + (u16)tmp2;
	return err;
}
static int ft5402_set_ky(struct i2c_client *client, u16 value)
{
	int err = 0;
	err = ft5402_write_reg(client, FT5402_REG_KY_H,
			value >> 8);
	if (err < 0)
		dev_err(&client->dev, "%s:set ky high failed\n",
				__func__);
	err = ft5402_write_reg(client, FT5402_REG_KY_L,
			value);
	if (err < 0)
		dev_err(&client->dev, "%s:set ky low failed\n",
				__func__);

	return err;
}

static int ft5402_get_ky(struct i2c_client *client, u16 *pvalue)
{
	int err = 0;
	u8 tmp1, tmp2;
	err = ft5402_read_reg(client, FT5402_REG_KY_H,
			&tmp1);
	if (err < 0)
		dev_err(&client->dev, "%s:get ky high failed\n",
				__func__);
	err = ft5402_read_reg(client, FT5402_REG_KY_L,
			&tmp2);
	if (err < 0)
		dev_err(&client->dev, "%s:get ky low failed\n",
				__func__);

	*pvalue = ((u16)tmp1<<8) + (u16)tmp2;
	return err;
}
static int ft5402_set_lemda_x(struct i2c_client *client, u8 value)
{
	return ft5402_write_reg(client, FT5402_REG_LEMDA_X,
			value);
}

static int ft5402_get_lemda_x(struct i2c_client *client, u8 *pvalue)
{
	return ft5402_read_reg(client, FT5402_REG_LEMDA_X,
			pvalue);
}
static int ft5402_set_lemda_y(struct i2c_client *client, u8 value)
{
	return ft5402_write_reg(client, FT5402_REG_LEMDA_Y,
			value);
}

static int ft5402_get_lemda_y(struct i2c_client *client, u8 *pvalue)
{
	return ft5402_read_reg(client, FT5402_REG_LEMDA_Y,
			pvalue);
}
static int ft5402_set_pos_x(struct i2c_client *client, u8 value)
{
	return ft5402_write_reg(client, FT5402_REG_DIRECTION,
			value);
}

static int ft5402_get_pos_x(struct i2c_client *client, u8 *pvalue)
{
	return ft5402_read_reg(client, FT5402_REG_DIRECTION,
			pvalue);
}

static int ft5402_set_scan_select(struct i2c_client *client, u8 value)
{
	return ft5402_write_reg(client, FT5402_REG_SCAN_SELECT,
			value);
}

static int ft5402_get_scan_select(struct i2c_client *client, u8 *pvalue)
{
	return ft5402_read_reg(client, FT5402_REG_SCAN_SELECT,
			pvalue);
}

static int ft5402_set_other_param(struct i2c_client *client)
{
	int err = 0;
	//err = ft5402_write_reg(client, FT5402_REG_THGROUP, (u8)(g_param_ft5402.ft5402_THGROUP));
	err = ft5402_write_reg(client, FT5402_REG_THGROUP, g_param_ft5402.ft5402_THGROUP);
	if (err < 0) {
		dev_err(&client->dev, "%s:write THGROUP failed.\n", __func__);
		return err;
	}
	err = ft5402_write_reg(client, FT5402_REG_THPEAK, g_param_ft5402.ft5402_THPEAK);
	if (err < 0) {
		dev_err(&client->dev, "%s:write THPEAK failed.\n",
				__func__);
		return err;
	}
	
	err = ft5402_write_reg(client, FT5402_REG_PWMODE_CTRL, 
		g_param_ft5402.ft5402_PWMODE_CTRL);
	if (err < 0) {
		dev_err(&client->dev, "%s:write PERIOD_CTRL failed.\n", __func__);
		return err;
	}
	err = ft5402_write_reg(client, FT5402_REG_PERIOD_ACTIVE,
			g_param_ft5402.ft5402_PERIOD_ACTIVE);
	if (err < 0) {
		dev_err(&client->dev, "%s:write PERIOD_ACTIVE failed.\n", __func__);
		return err;
	}
	
	err = ft5402_write_reg(client, FT5402_REG_FACE_DETECT_STATISTICS_TX_NUM,
			g_param_ft5402.ft5402_FACE_DETECT_STATISTICS_TX_NUM);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FACE_DETECT_STATISTICS_TX_NUM failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_MAX_TOUCH_VALUE_HIGH,
			g_param_ft5402.ft5402_MAX_TOUCH_VALUE>>8);
	if (err < 0) {
		dev_err(&client->dev, "%s:write MAX_TOUCH_VALUE_HIGH failed.\n", __func__);
		return err;
	}
	err = ft5402_write_reg(client, FT5402_REG_MAX_TOUCH_VALUE_LOW,
			g_param_ft5402.ft5402_MAX_TOUCH_VALUE);
	if (err < 0) {
		dev_err(&client->dev, "%s:write MAX_TOUCH_VALUE_LOW failed.\n", __func__);
		return err;
	}
	
	err = ft5402_write_reg(client, FT5402_REG_FACE_DETECT_MODE,
			g_param_ft5402.ft5402_FACE_DETECT_MODE);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FACE_DETECT_MODE failed.\n", __func__);
		return err;
	}
	err = ft5402_write_reg(client, FT5402_REG_DRAW_LINE_TH,
			g_param_ft5402.ft5402_DRAW_LINE_TH);
	if (err < 0) {
		dev_err(&client->dev, "%s:write DRAW_LINE_TH failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_POINTS_SUPPORTED,
			g_param_ft5402.ft5402_POINTS_SUPPORTED);
	if (err < 0) {
		dev_err(&client->dev, "%s:write POINTS_SUPPORTED failed.\n", __func__);
		return err;
	}
	err = ft5402_write_reg(client, FT5402_REG_ESD_FILTER_FRAME,
			g_param_ft5402.ft5402_ESD_FILTER_FRAME);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_ESD_FILTER_FRAME failed.\n", __func__);
		return err;
	}
	
	err = ft5402_write_reg(client, FT5402_REG_FT5402_POINTS_STABLE_MACRO,
			g_param_ft5402.ft5402_POINTS_STABLE_MACRO);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_POINTS_STABLE_MACRO failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_MIN_DELTA_X,
			g_param_ft5402.ft5402_MIN_DELTA_X);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_MIN_DELTA_X failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_MIN_DELTA_Y,
			g_param_ft5402.ft5402_MIN_DELTA_Y);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_MIN_DELTA_Y failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_MIN_DELTA_STEP,
			g_param_ft5402.ft5402_MIN_DELTA_STEP);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_MIN_DELTA_STEP failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_ESD_NOISE_MACRO,
			g_param_ft5402.ft5402_ESD_NOISE_MACRO);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_ESD_NOISE_MACRO failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_ESD_DIFF_VAL,
			g_param_ft5402.ft5402_ESD_DIFF_VAL);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_ESD_DIFF_VAL failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_ESD_NEGTIVE,
			g_param_ft5402.ft5402_ESD_NEGTIVE);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_ESD_NEGTIVE failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_ESD_FILTER_FRAMES,
			g_param_ft5402.ft5402_ESD_FILTER_FRAMES);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_ESD_FILTER_FRAMES failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_IO_LEVEL_SELECT,
			g_param_ft5402.ft5402_IO_LEVEL_SELECT);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_IO_LEVEL_SELECT failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_POINTID_DELAY_COUNT,
			g_param_ft5402.ft5402_POINTID_DELAY_COUNT);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_POINTID_DELAY_COUNT failed.\n", __func__);
		return err;
	}	

	err = ft5402_write_reg(client, FT5402_REG_FT5402_LIFTUP_FILTER_MACRO,
			g_param_ft5402.ft5402_LIFTUP_FILTER_MACRO);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_LIFTUP_FILTER_MACRO failed.\n", __func__);
		return err;
	}	

	err = ft5402_write_reg(client, FT5402_REG_FT5402_DIFF_HANDLE_MACRO,
			g_param_ft5402.ft5402_DIFF_HANDLE_MACRO);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_DIFF_HANDLE_MACRO failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_MIN_WATER,
			g_param_ft5402.ft5402_MIN_WATER);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_MIN_WATER failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_MAX_NOISE,
			g_param_ft5402.ft5402_MAX_NOISE);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_MAX_NOISE failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_WATER_START_RX,
			g_param_ft5402.ft5402_WATER_START_RX);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_WATER_START_RX failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_WATER_START_TX,
			g_param_ft5402.ft5402_WATER_START_TX);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_WATER_START_TX failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_HOST_NUMBER_SUPPORTED_MACRO,
			g_param_ft5402.ft5402_HOST_NUMBER_SUPPORTED_MACRO);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_HOST_NUMBER_SUPPORTED_MACRO failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_RAISE_THGROUP,
			g_param_ft5402.ft5402_RAISE_THGROUP);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_RAISE_THGROUP failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_CHARGER_STATE,
			g_param_ft5402.ft5402_CHARGER_STATE);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_CHARGER_STATE failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_FILTERID_START,
			g_param_ft5402.ft5402_FILTERID_START);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_FILTERID_START failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_FRAME_FILTER_EN_MACRO,
			g_param_ft5402.ft5402_FRAME_FILTER_EN_MACRO);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_FRAME_FILTER_EN_MACRO failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_FRAME_FILTER_SUB_MAX_TH,
			g_param_ft5402.ft5402_FRAME_FILTER_SUB_MAX_TH);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_FRAME_FILTER_SUB_MAX_TH failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_FRAME_FILTER_ADD_MAX_TH,
			g_param_ft5402.ft5402_FRAME_FILTER_ADD_MAX_TH);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_FRAME_FILTER_ADD_MAX_TH failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_FRAME_FILTER_SKIP_START_FRAME,
			g_param_ft5402.ft5402_FRAME_FILTER_SKIP_START_FRAME);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_FRAME_FILTER_SKIP_START_FRAME failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_FRAME_FILTER_BAND_EN,
			g_param_ft5402.ft5402_FRAME_FILTER_BAND_EN);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_FRAME_FILTER_BAND_EN failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_FT5402_FRAME_FILTER_BAND_WIDTH,
			g_param_ft5402.ft5402_FRAME_FILTER_BAND_WIDTH);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_FRAME_FILTER_BAND_WIDTH failed.\n", __func__);
		return err;
	}


	//mbg ++ 20131120
	err = ft5402_write_reg(client, FT5402_REG_POWER_NOISE_FILTER_PROCESS_EN,
			g_param_ft5402.ft5402_POWER_NOISE_FILTER_PROCESS_EN);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_POWER_NOISE_FILTER_PROCESS_EN failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_POWER_NOISE_RX_PEAK_DIFF,
			g_param_ft5402.ft5402_POWER_NOISE_RX_PEAK_DIFF);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_POWER_NOISE_RX_PEAK_DIFF failed.\n", __func__);
		return err;
	}

	err = ft5402_write_reg(client, FT5402_REG_POWER_NOISE_RX_NUM,
			g_param_ft5402.ft5402_POWER_NOISE_RX_NUM);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_POWER_NOISE_RX_NUM failed.\n", __func__);
		return err;
	}
	//mbg --
	return err;
}

static int ft5402_get_other_param(struct i2c_client *client)
{
	int err = 0;
	u8 value = 0x00;
	err = ft5402_read_reg(client, FT5402_REG_THGROUP, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write THGROUP failed.\n", __func__);
		return err;
	} else 
		DBG("THGROUP=%02x\n", value);
	err = ft5402_read_reg(client, FT5402_REG_THPEAK, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write THPEAK failed.\n",
				__func__);
		return err;
	} else 
		DBG("THPEAK=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_PWMODE_CTRL, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write PWMODE_CTRL failed.\n", __func__);
		return err;
	} else 
		DBG("CTRL=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_PERIOD_ACTIVE,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write PERIOD_ACTIVE failed.\n", __func__);
		return err;
	} else 
		DBG("PERIOD_ACTIVE=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_MAX_TOUCH_VALUE_HIGH,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write MAX_TOUCH_VALUE_HIGH failed.\n", __func__);
		return err;
	} else 
		DBG("MAX_TOUCH_VALUE_HIGH=%02x\n", value);
	err = ft5402_read_reg(client, FT5402_REG_MAX_TOUCH_VALUE_LOW,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write MAX_TOUCH_VALUE_LOW failed.\n", __func__);
		return err;
	} else 
		DBG("MAX_TOUCH_VALUE_LOW=%02x\n", value);
	err = ft5402_read_reg(client, FT5402_REG_FACE_DETECT_MODE,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FACE_DETECT_MODE failed.\n", __func__);
		return err;
	} else 
		DBG("FACE_DEC_MODE=%02x\n", value);
	err = ft5402_read_reg(client, FT5402_REG_DRAW_LINE_TH,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write DRAW_LINE_TH failed.\n", __func__);
		return err;
	} else 
		DBG("DRAW_LINE_TH=%02x\n", value);
	err = ft5402_read_reg(client, FT5402_REG_POINTS_SUPPORTED,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ft5402_POINTS_SUPPORTED failed.\n", __func__);
		return err;
	} else 
		DBG("ft5402_POINTS_SUPPORTED=%02x\n", value);
	err = ft5402_read_reg(client, FT5402_REG_ESD_FILTER_FRAME,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_ESD_FILTER_FRAME failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_ESD_FILTER_FRAME=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_POINTS_STABLE_MACRO,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_POINTS_STABLE_MACRO failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_POINTS_STABLE_MACRO=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_MIN_DELTA_X,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_MIN_DELTA_X failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_MIN_DELTA_X=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_MIN_DELTA_Y,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_MIN_DELTA_Y failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_MIN_DELTA_Y=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_MIN_DELTA_STEP,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_MIN_DELTA_STEP failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_MIN_DELTA_STEP=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_ESD_NOISE_MACRO,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_ESD_NOISE_MACRO failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_ESD_NOISE_MACRO=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_ESD_DIFF_VAL,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_ESD_DIFF_VAL failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_ESD_DIFF_VAL=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_ESD_NEGTIVE,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_ESD_NEGTIVE failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_ESD_NEGTIVE=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_ESD_FILTER_FRAMES,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_ESD_FILTER_FRAMES failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_ESD_FILTER_FRAMES=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_IO_LEVEL_SELECT,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_IO_LEVEL_SELECT failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_IO_LEVEL_SELECT=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_POINTID_DELAY_COUNT,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_POINTID_DELAY_COUNT failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_POINTID_DELAY_COUNT=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_LIFTUP_FILTER_MACRO,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_LIFTUP_FILTER_MACRO failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_LIFTUP_FILTER_MACRO=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_DIFF_HANDLE_MACRO,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_DIFF_HANDLE_MACRO failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_DIFF_HANDLE_MACRO=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_MIN_WATER,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_MIN_WATER failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_MIN_WATER=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_MAX_NOISE,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_MAX_NOISE failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_MAX_NOISE=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_WATER_START_RX,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_WATER_START_RX failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_WATER_START_RX=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_WATER_START_TX,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_WATER_START_TX failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_WATER_START_TX=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_HOST_NUMBER_SUPPORTED_MACRO,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_HOST_NUMBER_SUPPORTED_MACRO failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_HOST_NUMBER_SUPPORTED_MACRO=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_RAISE_THGROUP,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_RAISE_THGROUP failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_RAISE_THGROUP=%02x\n", value);
	
	err = ft5402_read_reg(client, FT5402_REG_FT5402_CHARGER_STATE,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_CHARGER_STATE failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_CHARGER_STATE=%02x\n", value);
	
	err = ft5402_read_reg(client, FT5402_REG_FT5402_FILTERID_START,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_FILTERID_START failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_FILTERID_START=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_FRAME_FILTER_EN_MACRO,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_FRAME_FILTER_EN_MACRO failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_FRAME_FILTER_EN_MACRO=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_FRAME_FILTER_SUB_MAX_TH,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_FRAME_FILTER_SUB_MAX_TH failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_FRAME_FILTER_SUB_MAX_TH=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_FRAME_FILTER_ADD_MAX_TH,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_FRAME_FILTER_ADD_MAX_TH failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_FRAME_FILTER_ADD_MAX_TH=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_FRAME_FILTER_SKIP_START_FRAME,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_FRAME_FILTER_SKIP_START_FRAME failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_FRAME_FILTER_SKIP_START_FRAME=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_FT5402_FRAME_FILTER_BAND_EN,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_FRAME_FILTER_BAND_EN failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_FRAME_FILTER_BAND_EN=%02x\n", value);
	
	err = ft5402_read_reg(client, FT5402_REG_FT5402_FRAME_FILTER_BAND_WIDTH,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_FT5402_FRAME_FILTER_BAND_WIDTH failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_FT5402_FRAME_FILTER_BAND_WIDTH=%02x\n", value);

	//mbg ++ 20131120
	err = ft5402_read_reg(client, FT5402_REG_POWER_NOISE_FILTER_PROCESS_EN,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_POWER_NOISE_FILTER_PROCESS_EN failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_POWER_NOISE_FILTER_PROCESS_EN=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_POWER_NOISE_RX_PEAK_DIFF,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_POWER_NOISE_RX_PEAK_DIFF failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_POWER_NOISE_RX_PEAK_DIFF=%02x\n", value);

	err = ft5402_read_reg(client, FT5402_REG_POWER_NOISE_RX_NUM,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:write FT5402_REG_POWER_NOISE_RX_NUM failed.\n", __func__);
		return err;
	} else 
		DBG("FT5402_REG_POWER_NOISE_RX_NUM=%02x\n", value);
	//mbg --
		
	return err;
}
int ft5402_get_ic_param(struct i2c_client *client)
{
	int err = 0;
	int i = 0;
	u8 value = 0x00;
	u16 xvalue = 0x0000, yvalue = 0x0000;
	
	/*enter factory mode*/
	err = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE, FT5402_FACTORYMODE_VALUE);
	if (err < 0) {
		dev_err(&client->dev, "%s:enter factory mode failed.\n", __func__);
		goto RETURN_WORK;
	}
	
	for (i = 0; i < g_ft5402_tx_num; i++) {
		DBG("tx%d:", i);
		/*get tx order*/
		err = ft5402_get_tx_order(client, i, &value);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not get tx%d order.\n",
					__func__, i);
			goto RETURN_WORK;
		}
		DBG("order=%d ", value);
		/*get tx cap*/
		err = ft5402_get_tx_cap(client, i, &value);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not get tx%d cap.\n",
					__func__, i);
			goto RETURN_WORK;
		}
		DBG("cap=%02x\n", value);
	}
	/*get tx offset*/
	err = ft5402_get_tx_offset(client, 0, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get tx 0 offset.\n",
				__func__);
		goto RETURN_WORK;
	} else
		DBG("tx offset = %02x\n", value);

	/*get rx offset and cap*/
	for (i = 0; i < g_ft5402_rx_num; i++) {
		/*get rx order*/
		DBG("rx%d:", i);
		err = ft5402_get_rx_order(client, i, &value);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not get rx%d order.\n",
					__func__, i);
			goto RETURN_WORK;
		}
		DBG("order=%d ", value);
		/*get rx cap*/
		err = ft5402_get_rx_cap(client, i, &value);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not get rx%d cap.\n",
					__func__, i);
			goto RETURN_WORK;
		}
		DBG("cap=%02x ", value);
		err = ft5402_get_rx_offset(client, i, &value);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not get rx offset.\n",
				__func__);
			goto RETURN_WORK;
		}
		DBG("offset=%02x\n", value);
	}

	/*get scan select*/
	err = ft5402_get_scan_select(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get scan select.\n",
			__func__);
		goto RETURN_WORK;
	} else
		DBG("scan select = %02x\n", value);
	
	/*get tx number*/
	err = ft5402_get_tx_num(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get tx num.\n",
			__func__);
		goto RETURN_WORK;
	} else
		DBG("tx num = %02x\n", value);
	/*get rx number*/
	err = ft5402_get_rx_num(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get rx num.\n",
			__func__);
		goto RETURN_WORK;
	} else
		DBG("rx num = %02x\n", value);
	
	/*get gain*/
	err = ft5402_get_gain(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get gain.\n",
			__func__);
		goto RETURN_WORK;
	} else
		DBG("gain = %02x\n", value);
	/*get voltage*/
	err = ft5402_get_vol(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get voltage.\n",
			__func__);
		goto RETURN_WORK;
	} else
		DBG("voltage = %02x\n", value);
	/*get start rx*/
	err = ft5402_get_start_rx(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get start rx.\n",
			__func__);
		goto RETURN_WORK;
	} else
		DBG("start rx = %02x\n", value);
	/*get adc target*/
	err = ft5402_get_adc_target(client, &xvalue);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get adc target.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("kx = %02x\n", xvalue);
	
	
RETURN_WORK:	
	/*enter work mode*/
	err = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE, FT5402_WORKMODE_VALUE);
	if (err < 0) {
		dev_err(&client->dev, "%s:enter work mode failed.\n", __func__);
		goto ERR_EXIT;
	}

	/*get resolution*/
	err = ft5402_get_Resolution(client, &xvalue, &yvalue);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get resolution.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("resolution X = %d Y = %d\n", xvalue, yvalue);


	/*get face detect statistics tx num*/
	err = ft5402_get_face_detect_statistics_tx_num(client,
			&value);
	if (err < 0) {
		dev_err(&client->dev,
				"%s:could not get face detect statistics tx num.\n",
				__func__);
		goto ERR_EXIT;
	} else
		DBG("FT5402_FACE_DETECT_STATISTICS_TX_NUM = %02x\n", value);
	/*get face detect pre value*/
	err = ft5402_get_face_detect_pre_value(client,
			&value);
	if (err < 0) {
		dev_err(&client->dev,
				"%s:could not get face detect pre value.\n",
				__func__);
		goto ERR_EXIT;
	} else
		DBG("FT5402_FACE_DETECT_PRE_VALUE = %02x\n", value);
	/*get face detect num*/
	err = ft5402_get_face_detect_num(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get face detect num.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("face detect num = %02x\n", value);

	/*get min peak value*/
	err = ft5402_get_peak_value_min(client,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get min peak value.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("FT5402_BIGAREA_PEAK_VALUE_MIN = %02x\n", value);
	/*get diff value over num*/
	err = ft5402_get_diff_value_over_num(client,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get diff value over num.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("FT5402_BIGAREA_DIFF_VALUE_OVER_NUM = %02x\n", value);
	/*get customer id*/
	err = ft5402_get_customer_id(client,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get customer id.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("FT5402_CUSTOMER_ID = %02x\n", value);	
	/*get kx*/
	err = ft5402_get_kx(client, &xvalue);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get kx.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("kx = %02x\n", xvalue);
	/*get ky*/
	err = ft5402_get_ky(client, &xvalue);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get ky.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("ky = %02x\n", xvalue);
	/*get lemda x*/
	err = ft5402_get_lemda_x(client,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get lemda x.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("lemda x = %02x\n", value);
	/*get lemda y*/
	err = ft5402_get_lemda_y(client,
			&value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get lemda y.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("lemda y = %02x\n", value);
	/*get pos x*/
	err = ft5402_get_pos_x(client, &value);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not get pos x.\n",
			__func__);
		goto ERR_EXIT;
	} else
		DBG("pos x = %02x\n", value);

	err = ft5402_get_other_param(client);
	
ERR_EXIT:
	return err;
}
EXPORT_SYMBOL_GPL(ft5402_get_ic_param);


int ft5402_Init_IC_Param(struct i2c_client *client)
{
	int err = 0;
	int i = 0;

	//mbg ++ 20130418_20:11
	script_item_u   val;
        
    if(SCIRPT_ITEM_VALUE_TYPE_INT != script_get_item("ctp_para", "ctp_ft5402_num", &val))
	{
        ctp_ft5402_num = 0;
	}
	else
    {
    	ctp_ft5402_num = val.val;
    }
    

	printk("+++++++++++++++++++++++++++++++++++++++++\n\n");

	
	//ctp_ft5402_num = 301;		//9022;
	printk("ctp_ft5402_num = %d\n", ctp_ft5402_num);
	
	switch(ctp_ft5402_num)
	{
		case 1:
			printk("project:D901LC\n");
			printk("Charge:XHY\n");
			printk("LCD:800 480\n");
			printk("TP:INET BOM\n");
			printk("Author:mbgalex\n");
			printk("Date:2013-11-28_10:28\n");
			//K901LC used
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	138;
			FT5402_KY		=	124;
			FT5402_RESOLUTION_X 	=	480;
			FT5402_RESOLUTION_Y 	=	800;
			FT5402_LEMDA_X		=	3;
			FT5402_LEMDA_Y		=	3;
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		25;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		2;
			FT5402_MIN_DELTA_Y		=	2;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	4;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	2;
			FT5402_FRAME_FILTER_EN		=	0;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 14;
			g_ft5402_gain= 21;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 8;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_K901LC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_K901LC[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
			unsigned char g_ft5402_rx_order_used_K901LC[] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_K901LC[] ={68,68,68,51,67,68,68};
			unsigned char g_ft5402_rx_cap_used_K901LC[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100};

			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_K901LC), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_K901LC, sizeof(g_ft5402_tx_order_used_K901LC));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_K901LC), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_K901LC, sizeof(g_ft5402_tx_cap_used_K901LC));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_K901LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_K901LC, sizeof(g_ft5402_rx_order_used_K901LC));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_K901LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_K901LC, sizeof(g_ft5402_rx_offset_used_K901LC));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_K901LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_K901LC, sizeof(g_ft5402_rx_cap_used_K901LC));
			break;
		
		case 2:
			printk("project:D801HC\n");
			printk("Charge:XHY\n");
			printk("LCD:1024 768\n");
			printk("TP:INET BOM\n");
			printk("Author:mbgalex\n");
			printk("Date:2013-11-28_10:28\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	193;
			FT5402_KY		=	159; 
			FT5402_RESOLUTION_X 	=	768;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	2; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		25;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		2;
			FT5402_MIN_DELTA_Y		=	2;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL 	=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME 	=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION	=	1;
			FT5402_POINTID_DELAY_COUNT		=	4;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL	=	-50;
			FT5402_MAX_NOISE_VAL	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	2;
			FT5402_FRAME_FILTER_EN		=	0;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN 	=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID 	=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 20;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 8;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D801HC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_D801HC[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
			unsigned char g_ft5402_rx_order_used_D801HC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D801HC[] = {67,52,52,68,51,66,51,51};
			unsigned char g_ft5402_rx_cap_used_D801HC[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
	
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D801HC), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D801HC, sizeof(g_ft5402_tx_order_used_D801HC));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D801HC), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D801HC, sizeof(g_ft5402_tx_cap_used_D801HC));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D801HC), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D801HC, sizeof(g_ft5402_rx_order_used_D801HC));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D801HC), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D801HC, sizeof(g_ft5402_rx_offset_used_D801HC));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D801HC), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D801HC, sizeof(g_ft5402_rx_cap_used_D801HC));
			break;
		
		case 3:
			printk("project:D801LC\n");
			printk("Charge:XHY\n");
			printk("LCD:800 600\n");
			printk("TP:INET BOM\n");
			printk("Author:mbgalex\n");
			printk("Date:2013-12-04_18:59\n");
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	124; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	800;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	5; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		21;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		3;
			FT5402_MIN_DELTA_Y		=	3;
			FT5402_MIN_DELTA_STEP	=		3;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	4;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	2;
			FT5402_FRAME_FILTER_EN		=	0;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID		=	0;

			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 19;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 8;
			g_ft5402_tx_offset = 0;

			unsigned char g_ft5402_tx_order_used_K801LC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_K801LC[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
			unsigned char g_ft5402_rx_order_used_K801LC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_K801LC[] = {52,68,67,52,68,68,68,67};
			unsigned char g_ft5402_rx_cap_used_K801LC[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};

			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_K801LC), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_K801LC, sizeof(g_ft5402_tx_order_used_K801LC));

			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_K801LC), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_K801LC, sizeof(g_ft5402_tx_cap_used_K801LC));

			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_K801LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_K801LC, sizeof(g_ft5402_rx_order_used_K801LC));

			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_K801LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_K801LC, sizeof(g_ft5402_rx_offset_used_K801LC));

			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_K801LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_K801LC, sizeof(g_ft5402_rx_cap_used_K801LC));
			break;


	#if 1
	
		case 301:
			printk("project:D80\n");
		    printk("clientele:ZHONG DIAN\n");
		    printk("Charge:PGAD0500200W1EU\n");
		    printk("LCD:\n");
		    printk("TP:FHX\n");
		    printk("Author:ZENG QING MING\n");
		    printk("Date:2014-04-17\n");

			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;  
			FT5402_KY		=	125;
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	800;
			FT5402_LEMDA_X		=	0;
			FT5402_LEMDA_Y		=	0;
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		22;
			FT5402_THPEAK		=	80;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		4;
			FT5402_MIN_DELTA_Y		=	4;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL 	=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME 	=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION	=	1;
			FT5402_POINTID_DELAY_COUNT		=	6;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL	=	-50;
			FT5402_MAX_NOISE_VAL	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	5;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	5;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN 	=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID 	=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 14;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 6;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			//unsigned char g_ft5402_tx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU[] = {60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60};
    
	        unsigned char g_ft5402_tx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU[] = {60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85};

			unsigned char g_ft5402_rx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU[] ={34,50,34,34,33,35,50,34};
			unsigned char g_ft5402_rx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU[] = {140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140};
	
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU, sizeof(g_ft5402_tx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU, sizeof(g_ft5402_tx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU, sizeof(g_ft5402_rx_order_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU, sizeof(g_ft5402_rx_offset_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU, sizeof(g_ft5402_rx_cap_used_K100_ZHONGDIAN_FHX_PGAD0500200W1EU));
			break;
	
#endif
		case 4:
			printk("project:D100lLC\n");
			printk("Charge:XHY\n");
			printk("LCD:1024 600\n");
			printk("TP:INET BOM\n");
			printk("Author:mbgalex\n");
			printk("Date:2013-11-28_10:28\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		22;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		3;
			FT5402_MIN_DELTA_Y		=	3;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	2;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	2;
			FT5402_FRAME_FILTER_EN		=	0;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 16;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 8;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_K1001LC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_K1001LC[] = {36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,60,62,64,66,72,78,84,90,96,102,108};
			unsigned char g_ft5402_rx_order_used_K1001LC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_K1001LC[] = {67,67,83,68,52,67,68,67};
			unsigned char g_ft5402_rx_cap_used_K1001LC[] = {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120};

			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_K1001LC), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_K1001LC, sizeof(g_ft5402_tx_order_used_K1001LC));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_K1001LC), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_K1001LC, sizeof(g_ft5402_tx_cap_used_K1001LC));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_K1001LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_K1001LC, sizeof(g_ft5402_rx_order_used_K1001LC));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_K1001LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_K1001LC, sizeof(g_ft5402_rx_offset_used_K1001LC));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_K1001LC), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_K1001LC, sizeof(g_ft5402_rx_cap_used_K1001LC));
			break;
		#if 0
		case 401:
			printk("project:D100lLC\n");
			printk("client:GeRuiBang");
			printk("Charge:Unknow\n");
			printk("LCD:1024 600\n");
			printk("TP:DPT\n");
			printk("Author:zhangshengxian\n");
			printk("Date:2014-01-13_17:37\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		22;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		3;//5;
			FT5402_MIN_DELTA_Y		=	3;//5;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	2;//6;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		1;
			FT5402_RV_G_RAISE_THGROUP		=	70;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	0;//1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;//15;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;//15;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;//2;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_POWER_NOISE_FILTER_PROCESS_EN	=	1;
			FT5402_POWER_NOISE_RX_PEAK_DIFF	=	70;//100;
			FT5402_POWER_NOISE_RX_NUM	=	6;//7;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 16;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 8; //14;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D100_GRB[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_D100_GRB[] = {36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,60,62,64,66,72,78,84,90,96,102,108};//{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};//{36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,60,62,64,66,72,78,84,90,96,102,108};
			unsigned char g_ft5402_rx_order_used_D100_GRB[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D100_GRB[] = {67,35,66,51,69,67,68,38};//{52,67,67,51,52,51,51,52};
			unsigned char g_ft5402_rx_cap_used_D100_GRB[] = {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120};
			
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D100_GRB), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D100_GRB, sizeof(g_ft5402_tx_order_used_D100_GRB));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D100_GRB), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D100_GRB, sizeof(g_ft5402_tx_cap_used_D100_GRB));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D100_GRB), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D100_GRB, sizeof(g_ft5402_rx_order_used_D100_GRB));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D100_GRB), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D100_GRB, sizeof(g_ft5402_rx_offset_used_D100_GRB));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D100_GRB), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D100_GRB, sizeof(g_ft5402_rx_cap_used_D100_GRB));
			break;
		#endif
		
		#if 0			
			case 402:
			printk("project:D100\n");
			printk("Charge:CHUANG JIA\n");
			printk("Adaptor:JHD-AP012E-050200AA\n");
			printk("TP:YUN TIAN GE\n");
			printk("Author:ZENGQINGMING\n");
			printk("Date:2014-01-09\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		20;//23;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		5;
			FT5402_MIN_DELTA_Y		=	5;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	2;//6;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	12;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		1;
			FT5402_RV_G_RAISE_THGROUP		=	40;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	15;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	15;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		2;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_POWER_NOISE_FILTER_PROCESS_EN	=	0;
			FT5402_POWER_NOISE_RX_PEAK_DIFF	=	90;
			FT5402_POWER_NOISE_RX_NUM	=	2;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 19; 
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 14;//5;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			//unsigned char g_ft5402_tx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {16,20,25,30,35,44,42,43,44,45,46,47,48,49,50,60,62,64,66,72,78,84,90,96,102,108};
			
			
			//unsigned char g_ft5402_tx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {5,5,7,9,10,13,16,21,24,28,30,32,40,50,60,70,80,90,100,110,120,130,140,140,150,150};
			
			unsigned char g_ft5402_tx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,95};
			
			unsigned char g_ft5402_rx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {2,84,82,34,35,51,68,51};//{67,67,83,68,52,67,68,67};
			//unsigned char g_ft5402_rx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};

      		unsigned char g_ft5402_rx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA[] = {160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};
	
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA, sizeof(g_ft5402_tx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA, sizeof(g_ft5402_tx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA, sizeof(g_ft5402_rx_order_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA, sizeof(g_ft5402_rx_offset_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA, sizeof(g_ft5402_rx_cap_used_D100_CHUANGJIA_YUNTIANGE_JHDAP012E050200AA));
			break;
	#endif

		#if 0
		case 403:
			printk("project:D100lLC_Only for test Raysens TP\n");
			printk("Charge:PS14K0502000ES(格瑞邦D100项目用充电器)\n");
			printk("LCD:1024 600\n");
			printk("TP:RAYSENS\n");
			printk("Author:zhangshengxian\n");
			printk("Date:2014-01-13_17:37\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		18;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		5;
			FT5402_MIN_DELTA_Y		=	5;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL 	=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME 	=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION	=	1;
			FT5402_POINTID_DELAY_COUNT		=	2;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL	=	-50;
			FT5402_MAX_NOISE_VAL	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		1;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	0;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN 	=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_POWER_NOISE_FILTER_PROCESS_EN	=	0;
			FT5402_POWER_NOISE_RX_PEAK_DIFF =	70;
			FT5402_POWER_NOISE_RX_NUM	=	6;
			FT5402_OTP_PARAM_ID 	=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 17;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 4;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D100_PW_RAYSENS_PS14K0502000ES[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_D100_PW_RAYSENS_PS14K0502000ES[] = {10,11,14,17,20,23,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,74,77,80,95};
			unsigned char g_ft5402_rx_order_used_D100_PW_RAYSENS_PS14K0502000ES[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D100_PW_RAYSENS_PS14K0502000ES[] = {67,35,66,51,69,67,68,38};
			unsigned char g_ft5402_rx_cap_used_D100_PW_RAYSENS_PS14K0502000ES[] = {160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};
			
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D100_PW_RAYSENS_PS14K0502000ES), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D100_PW_RAYSENS_PS14K0502000ES, sizeof(g_ft5402_tx_order_used_D100_PW_RAYSENS_PS14K0502000ES));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D100_PW_RAYSENS_PS14K0502000ES), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D100_PW_RAYSENS_PS14K0502000ES, sizeof(g_ft5402_tx_cap_used_D100_PW_RAYSENS_PS14K0502000ES));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D100_PW_RAYSENS_PS14K0502000ES), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D100_PW_RAYSENS_PS14K0502000ES, sizeof(g_ft5402_rx_order_used_D100_PW_RAYSENS_PS14K0502000ES));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D100_PW_RAYSENS_PS14K0502000ES), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D100_PW_RAYSENS_PS14K0502000ES, sizeof(g_ft5402_rx_offset_used_D100_PW_RAYSENS_PS14K0502000ES));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D100_PW_RAYSENS_PS14K0502000ES), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D100_PW_RAYSENS_PS14K0502000ES, sizeof(g_ft5402_rx_cap_used_D100_PW_RAYSENS_PS14K0502000ES));
			break;


			#endif

	#if 0
		case 404:
			printk("project:D100lLC\n");
			printk("client:chang jia\n");
			printk("Charge:JHKAP012E050200AA\n");
			printk("LCD:1024 600\n");
			printk("TP:RUI SHI\n");
			printk("Author:ZENG QING MING\n");
			printk("Date:2014-01-15\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		23;//24;
			FT5402_THPEAK		=	90;//90;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		4;
			FT5402_MIN_DELTA_Y		=	4;
			FT5402_MIN_DELTA_STEP	=		4;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	2;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	1;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	10;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	10;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 29;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 14;//14;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			//unsigned char g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {10,11,14,17,20,23,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,74,77,80,95};
             
			unsigned char g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60};
            
			//unsigned char g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {40,42,44,46,48,50,52,54,56,58,60,62,64,68,70,72,74,76,78,80,82,84,86,90,92,100};

			unsigned char g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {67,35,66,51,69,67,68,38};
			//unsigned char g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};


			unsigned char g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120};
			
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			break;
		#endif

	#if 0
		case 405:
			printk("project:D100lLC\n");
			printk("client:YI FU DA\n");
			printk("Charge:JKY502-050200EU\n");
			printk("LCD:1024 600\n");
			printk("TP:DE PU TE\n");
			printk("Author:ZENG QING MING\n");
			printk("Date:2014-01-15\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		18;//24;
			FT5402_THPEAK		=	70;//90;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		6;
			FT5402_MIN_DELTA_Y		=	6;
			FT5402_MIN_DELTA_STEP	=		4;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	6;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	1;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	8;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	8;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 29;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 13;//14;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			//unsigned char g_ft5402_tx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {10,11,14,17,20,23,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,74,77,80,95};
             
			unsigned char g_ft5402_tx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80};
            
			//unsigned char g_ft5402_tx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {40,42,44,46,48,50,52,54,56,58,60,62,64,68,70,72,74,76,78,80,82,84,86,90,92,100};

			unsigned char g_ft5402_rx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {67,35,66,51,69,67,68,38};
			unsigned char g_ft5402_rx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};


			//unsigned char g_ft5402_rx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU[] = {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120};
			
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU, sizeof(g_ft5402_tx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU, sizeof(g_ft5402_tx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU, sizeof(g_ft5402_rx_order_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU, sizeof(g_ft5402_rx_offset_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU, sizeof(g_ft5402_rx_cap_used_D1001C_YIFUDA_DEPUTE_JKY502050200EU));
			break;


			#endif
		#if 0
		case 406:
			printk("project:D100lLC\n");
			printk("client:chang jia\n");
			printk("Charge:JHKAP012E050200AA\n");
			printk("LCD:1024 600\n");
			printk("TP:yun tian ge\n");
			printk("Author:zhang sheng xian\n");
			printk("Date:2014-01-17\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		20;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		5;
			FT5402_MIN_DELTA_Y		=	5;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	2;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	10;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		1;
			FT5402_RV_G_RAISE_THGROUP		=	28;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	15;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	15;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		2;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_POWER_NOISE_FILTER_PROCESS_EN	=0;
			FT5402_POWER_NOISE_RX_PEAK_DIFF	=	90;
			FT5402_POWER_NOISE_RX_NUM	=	2;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 20;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 14;//14;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};

			unsigned char g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,95};

			unsigned char g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {51,35,67,18,99,67,51,2};

			unsigned char g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA[] = {160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};
			
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_tx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_tx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_rx_order_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_rx_offset_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA, sizeof(g_ft5402_rx_cap_used_D1001C_CHANGHONG_RS_JHKAP012E050200AA));
			break;
		#endif
#if 0
			case 407:

			printk("project:D901HC\n");
			printk("Charge:JJD\n");
			printk("LCD:1024 600\n");
			printk("TP:INET BOM\n");
			printk("Author:mbgalex\n");
			printk("Date:2014-01-22_10:28\n");

			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	152;
			FT5402_KY		=	160;
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	7;
			FT5402_LEMDA_Y		=	5;
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		24;//22;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		5;
			FT5402_MIN_DELTA_Y		=	5;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL 	=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME 	=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION	=	1;
			FT5402_POINTID_DELAY_COUNT		=	4;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL	=	-50;
			FT5402_MAX_NOISE_VAL	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	1;
			FT5402_RV_G_FILTERID_START		=	1;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	5;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	5;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		2;
			FT5402_FRAME_FILTER_BAND_EN 	=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID 	=	0;

			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 22;//19;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 3;//8;
			g_ft5402_tx_offset = 0;

			unsigned char g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};//{80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80};//{50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
			unsigned char g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {52,68,67,52,68,68,68,68};
			unsigned char g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};//{160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};//{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));
			break;


#endif
#if 1
case 9022:
			printk("RUI FU");
			printk("project:U901HC\n");
			printk("Charge:JJD\n");
			printk("LCD:1024 600\n");
			printk("TP:INET BOM\n");
			printk("Author:mbgalex\n");
			printk("Date:2014-01-22_10:28\n");

			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	157;
			FT5402_KY		=	155;
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	3;
			FT5402_LEMDA_Y		=	3;
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		25;//22;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		2;
			FT5402_MIN_DELTA_Y		=	2;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL 	=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME 	=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION	=	1;
			FT5402_POINTID_DELAY_COUNT		=	3;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL	=	-50;
			FT5402_MAX_NOISE_VAL	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	2;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	6;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	6;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN 	=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID 	=	0;

			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 21;//19;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 12;//8;
			g_ft5402_tx_offset = 0;

			unsigned char g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};//{80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80};//{50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
			unsigned char g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {53,52,68,52,68,68,85,67};
			unsigned char g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};//{160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};//{100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_tx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_tx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_rx_order_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_rx_offset_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));

			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU, sizeof(g_ft5402_rx_cap_used_D90C_YIDAISHUMA_YOUCHU_JF012WR_05020BU));
			break;

#endif
#if 1		
			
		case 408:
			printk("project:D901HC\n");
			printk("Charge:SUN-0500200\n");
			printk("LCD:1024 600\n");
			printk("TP:QSD\n");
			printk("Author:zhangshengxian\n");
			printk("Date:2014-02-18_17:17\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	152;
			FT5402_KY		=	160;
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	7;
			FT5402_LEMDA_Y		=	5;
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		18;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		4;
			FT5402_MIN_DELTA_Y		=	4;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL 	=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME 	=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION	=	1;
			FT5402_POINTID_DELAY_COUNT		=	2;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL	=	-50;
			FT5402_MAX_NOISE_VAL	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		1;
			FT5402_RV_G_RAISE_THGROUP		=	20;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	1;
			FT5402_FRAME_FILTER_EN		=	0;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
			FT5402_FRAME_FILTER_BAND_EN 	=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_POWER_NOISE_FILTER_PROCESS_EN	=	0;
			FT5402_POWER_NOISE_RX_PEAK_DIFF =	90;
			FT5402_POWER_NOISE_RX_NUM	=	2;
			FT5402_OTP_PARAM_ID 	=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 20;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 3;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D901HC[] ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_D901HC[] = {42,43,44,45,46,47,48,49,50,51,52,53,54,56,58,60,62,64,66,68,70,72,74,76,78,85};
			unsigned char g_ft5402_rx_order_used_D901HC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D901HC[] = {67,52,67,51,35,52,37,52};
			unsigned char g_ft5402_rx_cap_used_D901HC[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D901HC), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D901HC, sizeof(g_ft5402_tx_order_used_D901HC));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D901HC), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D901HC, sizeof(g_ft5402_tx_cap_used_D901HC));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D901HC), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D901HC, sizeof(g_ft5402_rx_order_used_D901HC));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D901HC), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D901HC, sizeof(g_ft5402_rx_offset_used_D901HC));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D901HC), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D901HC, sizeof(g_ft5402_rx_cap_used_D901HC));
			break;
			
#endif

#if 0
		case 409:
			printk("project:D901HC\n");
			printk("Charge:JHD-AP012E-050200AB\n");
			printk("LCD:1024 600\n");
			printk("TP:QSD\n");
			printk("Author:zhangshengxian\n");
			printk("Date:2014-03-16_18:22\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	152;
			FT5402_KY		=	160;
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	7;
			FT5402_LEMDA_Y		=	5;
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		20;//23;//18;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		4;
			FT5402_MIN_DELTA_Y		=	4;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL 	=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME 	=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION	=	1;
			FT5402_POINTID_DELAY_COUNT		=	4;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	12;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL	=	-50;
			FT5402_MAX_NOISE_VAL	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		1;
			FT5402_RV_G_RAISE_THGROUP		=	68; // 4;//40;(52 60 68)>=80
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	1;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	12;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	12;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		2;
			FT5402_FRAME_FILTER_BAND_EN 	=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_POWER_NOISE_FILTER_PROCESS_EN	=	1;
			FT5402_POWER_NOISE_RX_PEAK_DIFF =	170;
			FT5402_POWER_NOISE_RX_NUM	=	3;
			FT5402_OTP_PARAM_ID 	=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 18;//20;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect=4;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_D901HC_JHD_AP012E_050200AB[] ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_D901HC_JHD_AP012E_050200AB[] = {42,43,44,45,46,47,48,49,50,51,52,53,54,56,58,60,62,64,66,68,70,72,74,76,78,85};
			unsigned char g_ft5402_rx_order_used_D901HC_JHD_AP012E_050200AB[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_D901HC_JHD_AP012E_050200AB[] = {67,52,67,51,35,52,37,52};
			unsigned char g_ft5402_rx_cap_used_D901HC_JHD_AP012E_050200AB[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D901HC_JHD_AP012E_050200AB), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D901HC_JHD_AP012E_050200AB, sizeof(g_ft5402_tx_order_used_D901HC_JHD_AP012E_050200AB));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D901HC_JHD_AP012E_050200AB), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D901HC_JHD_AP012E_050200AB, sizeof(g_ft5402_tx_cap_used_D901HC_JHD_AP012E_050200AB));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D901HC_JHD_AP012E_050200AB), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D901HC_JHD_AP012E_050200AB, sizeof(g_ft5402_rx_order_used_D901HC_JHD_AP012E_050200AB));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D901HC_JHD_AP012E_050200AB), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D901HC_JHD_AP012E_050200AB, sizeof(g_ft5402_rx_offset_used_D901HC_JHD_AP012E_050200AB));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D901HC_JHD_AP012E_050200AB), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D901HC_JHD_AP012E_050200AB, sizeof(g_ft5402_rx_cap_used_D901HC_JHD_AP012E_050200AB));
			break;
			
#endif

#if 0
	case 410:
		printk("project:D100lLC\n");
		printk("client:TIAN ZHENG HONG YE\n");
		printk("Charge:TEKA012-050200EU\n");
		printk("LCD:1024 600\n");
		printk("TP:yun tian ge\n");
		printk("Author:ZENG QING MING\n");
		printk("Date:2014-03-7\n");
		
		FT5402_START_RX 	=	0;
		FT5402_ADC_TARGET		=	8500;
		FT5402_KX		=	151;
		FT5402_KY		=	158; 
		FT5402_RESOLUTION_X 	=	600;
		FT5402_RESOLUTION_Y 	=	1024;
		FT5402_LEMDA_X		=	0; 
		FT5402_LEMDA_Y		=	0; 
		FT5402_PWMODE_CTRL	=		1;
		FT5402_POINTS_SUPPORTED 	=	5;
		FT5402_DRAW_LINE_TH 	=	150;
		FT5402_FACE_DETECT_MODE 	=	0;
		FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
		FT5402_FACE_DETECT_PRE_VALUE	=		20;
		FT5402_FACE_DETECT_NUM		=	10;
		FT5402_THGROUP	=		22;
		FT5402_THPEAK		=	80;
		FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
		FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
		FT5402_MIN_DELTA_X	=		8;
		FT5402_MIN_DELTA_Y		=	8;
		FT5402_MIN_DELTA_STEP	=		2;
		FT5402_ESD_DIFF_VAL 	=	20;
		FT5402_ESD_NEGTIVE		=	-50;
		FT5402_ESD_FILTER_FRAME 	=	10;
		FT5402_MAX_TOUCH_VALUE		=	600;
		FT5402_CUSTOMER_ID		=	121;
		FT5402_IO_LEVEL_SELECT	=		0;
		FT5402_DIRECTION	=	1;
		FT5402_POINTID_DELAY_COUNT		=	6;
		FT5402_LIFTUP_FILTER_MACRO		=	0;
		FT5402_POINTS_STABLE_MACRO		=	1;
		FT5402_ESD_NOISE_MACRO		=	0;
		FT5402_RV_G_PERIOD_ACTIVE		=	14;
		FT5402_DIFFDATA_HANDLE		=	0;
		FT5402_MIN_WATER_VAL	=	-50;
		FT5402_MAX_NOISE_VAL	=	10;
		FT5402_WATER_HANDLE_START_RX =		0;
		FT5402_WATER_HANDLE_START_TX =		0;
		FT5402_HOST_NUMBER_SUPPORTED =		1;
		FT5402_RV_G_RAISE_THGROUP		=	28;
		FT5402_RV_G_CHARGER_STATE		=	0;
		FT5402_RV_G_FILTERID_START		=	0;
		FT5402_FRAME_FILTER_EN		=	1;
		FT5402_FRAME_FILTER_SUB_MAX_TH		=	8;
		FT5402_FRAME_FILTER_ADD_MAX_TH		=	8;
		FT5402_FRAME_FILTER_SKIP_START_FRAME =		8; 
		FT5402_FRAME_FILTER_BAND_EN 	=	1;
		FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
		FT5402_POWER_NOISE_FILTER_PROCESS_EN	=1;
		FT5402_POWER_NOISE_RX_PEAK_DIFF =	90;
		FT5402_POWER_NOISE_RX_NUM	=	2;
		FT5402_OTP_PARAM_ID 	=	0;
		
		g_ft5402_tx_num = 26;
		g_ft5402_rx_num = 16;
		g_ft5402_gain= 27;//23;
		g_ft5402_voltage = 4;
		g_ft5402_scanselect= 14;
		g_ft5402_tx_offset = 0;
		
		unsigned char g_ft5402_tx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};

		unsigned char g_ft5402_tx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU[] = {80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80};

		//unsigned char g_ft5402_tx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU[] = {20,23,26,29,32,35,35,41,44,47,50,53,56,59,62,65,68,71,74,77,80,83,86,89,92,95};

		unsigned char g_ft5402_rx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		unsigned char g_ft5402_rx_offset_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU[] = {51,35,67,18,99,67,51,2};

		unsigned char g_ft5402_rx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU[] = {160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160};
		
		g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU), GFP_KERNEL);
		memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU, sizeof(g_ft5402_tx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU));
		
		g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU), GFP_KERNEL);
		memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU, sizeof(g_ft5402_tx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU));
		
		g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU), GFP_KERNEL);
		memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU, sizeof(g_ft5402_rx_order_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU));
		
		g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU), GFP_KERNEL);
		memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU, sizeof(g_ft5402_rx_offset_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU));
		
		g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU), GFP_KERNEL);
		memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU, sizeof(g_ft5402_rx_cap_used_D1001C_TIANZHENGHONGYE_YUNTIANGE_TEKA012050200EU));
		break;
		#endif

	#if 0
	case 413:

		printk("K1001LC_DEYVFANG_QLT_XHY050200LA\n");
		FT5402_START_RX 	=	0;
		FT5402_ADC_TARGET		=	8500;
		FT5402_KX		=	151;
		FT5402_KY		=	158; 
		FT5402_RESOLUTION_X 	=	600;
		FT5402_RESOLUTION_Y 	=	1024;
		FT5402_LEMDA_X		=	5; 
		FT5402_LEMDA_Y		=	4; 
		FT5402_PWMODE_CTRL	=		1;
		FT5402_POINTS_SUPPORTED 	=	5;
		FT5402_DRAW_LINE_TH 	=	150;
		FT5402_FACE_DETECT_MODE 	=	0;
		FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
		FT5402_FACE_DETECT_PRE_VALUE	=		20;
		FT5402_FACE_DETECT_NUM		=	10;
		FT5402_THGROUP	=		27;
		FT5402_THPEAK		=	60;
		FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
		FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
		FT5402_MIN_DELTA_X	=		5;
		FT5402_MIN_DELTA_Y		=	5;
		FT5402_MIN_DELTA_STEP	=		2;
		FT5402_ESD_DIFF_VAL		=	20;
		FT5402_ESD_NEGTIVE		=	-50;
		FT5402_ESD_FILTER_FRAME		=	10;
		FT5402_MAX_TOUCH_VALUE		=	600;
		FT5402_CUSTOMER_ID		=	121;
		FT5402_IO_LEVEL_SELECT	=		0;
		FT5402_DIRECTION 	=	1;
		FT5402_POINTID_DELAY_COUNT		=	5;
		FT5402_LIFTUP_FILTER_MACRO		=	0;
		FT5402_POINTS_STABLE_MACRO		=	1;
		FT5402_ESD_NOISE_MACRO		=	0;
		FT5402_RV_G_PERIOD_ACTIVE		=	16;
		FT5402_DIFFDATA_HANDLE		=	0;
		FT5402_MIN_WATER_VAL 	=	-50;
		FT5402_MAX_NOISE_VAL 	=	10;
		FT5402_WATER_HANDLE_START_RX =		0;
		FT5402_WATER_HANDLE_START_TX =		0;
		FT5402_HOST_NUMBER_SUPPORTED =		0;
		FT5402_RV_G_RAISE_THGROUP		=	30;
		FT5402_RV_G_CHARGER_STATE		=	0;
		FT5402_RV_G_FILTERID_START		=	2;
		FT5402_FRAME_FILTER_EN		=	0;
		FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
		FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
		FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
		FT5402_FRAME_FILTER_BAND_EN		=	1;
		FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
		FT5402_OTP_PARAM_ID		=	0;
		
		g_ft5402_tx_num = 26;
		g_ft5402_rx_num = 16;
		g_ft5402_gain= 23;
		g_ft5402_voltage = 4;
		g_ft5402_scanselect= 7;//8;
		g_ft5402_tx_offset = 0;
		
		unsigned char g_ft5402_tx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
		unsigned char g_ft5402_tx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA[] = {36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,60,62,64,66,72,78,84,90,96,102,108};
		unsigned char g_ft5402_rx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		unsigned char g_ft5402_rx_offset_used_K1001LC_DEYVFANG_QLT_XHY050200LA[] = {67,67,83,68,52,67,68,67};
		unsigned char g_ft5402_rx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA[] = {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120};

		g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA), GFP_KERNEL);
		memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA, sizeof(g_ft5402_tx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA));
		
		g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA), GFP_KERNEL);
		memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA, sizeof(g_ft5402_tx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA));
		
		g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA), GFP_KERNEL);
		memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA, sizeof(g_ft5402_rx_order_used_K1001LC_DEYVFANG_QLT_XHY050200LA));
		
		g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_K1001LC_DEYVFANG_QLT_XHY050200LA), GFP_KERNEL);
		memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_K1001LC_DEYVFANG_QLT_XHY050200LA, sizeof(g_ft5402_rx_offset_used_K1001LC_DEYVFANG_QLT_XHY050200LA));
		
		g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA), GFP_KERNEL);
		memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA, sizeof(g_ft5402_rx_cap_used_K1001LC_DEYVFANG_QLT_XHY050200LA));
		break;
	#endif	

#if 0

		case 414:
			printk("project:D1001C\n");
			printk("Charge:SCX\n");
			printk("LCD:1024 600\n");
			printk("TP:HONGSHAN\n");
			printk("Author:mbgalex\n");
			printk("Date:2013-11-28_10:28\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		24;
			FT5402_THPEAK		=	70;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		5;
			FT5402_MIN_DELTA_Y		=	5;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	9;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	15;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	15;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		2;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 17;
			g_ft5402_voltage = 4;
			g_ft5402_scanselect= 5;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020[] = {30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30};//{36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,60,62,64,66,72,78,84,90,96,102,108};
			unsigned char g_ft5402_rx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020[] = {67,67,83,68,52,67,68,67};
			unsigned char g_ft5402_rx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020[] = {220,220,220,220,220,220,220,220,220,220,220,220,220,220,220,220};//{120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120};

			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020, sizeof(g_ft5402_tx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020, sizeof(g_ft5402_tx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020, sizeof(g_ft5402_rx_order_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020, sizeof(g_ft5402_rx_offset_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020, sizeof(g_ft5402_rx_cap_used_K1001LC_XINHANPAI_HONGSHAN_SCX_05020));
			break;

#endif

#if 0

		case 415:
			printk("project:D100lLC\n");
			printk("Charge:JKY\n");
			printk("LCD:1024 600\n");
			printk("TP:DPT\n");
			printk("Author:mbgalex\n");
			printk("Date:2014-4-9_10:28\n");
			
			FT5402_START_RX 	=	0;
			FT5402_ADC_TARGET		=	8500;
			FT5402_KX		=	151;
			FT5402_KY		=	158; 
			FT5402_RESOLUTION_X 	=	600;
			FT5402_RESOLUTION_Y 	=	1024;
			FT5402_LEMDA_X		=	5; 
			FT5402_LEMDA_Y		=	4; 
			FT5402_PWMODE_CTRL	=		1;
			FT5402_POINTS_SUPPORTED 	=	5;
			FT5402_DRAW_LINE_TH 	=	150;
			FT5402_FACE_DETECT_MODE 	=	0;
			FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
			FT5402_FACE_DETECT_PRE_VALUE	=		20;
			FT5402_FACE_DETECT_NUM		=	10;
			FT5402_THGROUP	=		27;//24;
			FT5402_THPEAK		=	60;
			FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
			FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
			FT5402_MIN_DELTA_X	=		5;
			FT5402_MIN_DELTA_Y		=	5;
			FT5402_MIN_DELTA_STEP	=		2;
			FT5402_ESD_DIFF_VAL		=	20;
			FT5402_ESD_NEGTIVE		=	-50;
			FT5402_ESD_FILTER_FRAME		=	10;
			FT5402_MAX_TOUCH_VALUE		=	600;
			FT5402_CUSTOMER_ID		=	121;
			FT5402_IO_LEVEL_SELECT	=		0;
			FT5402_DIRECTION 	=	1;
			FT5402_POINTID_DELAY_COUNT		=	10;
			FT5402_LIFTUP_FILTER_MACRO		=	0;
			FT5402_POINTS_STABLE_MACRO		=	1;
			FT5402_ESD_NOISE_MACRO		=	0;
			FT5402_RV_G_PERIOD_ACTIVE		=	16;
			FT5402_DIFFDATA_HANDLE		=	0;
			FT5402_MIN_WATER_VAL 	=	-50;
			FT5402_MAX_NOISE_VAL 	=	10;
			FT5402_WATER_HANDLE_START_RX =		0;
			FT5402_WATER_HANDLE_START_TX =		0;
			FT5402_HOST_NUMBER_SUPPORTED =		0;
			FT5402_RV_G_RAISE_THGROUP		=	30;
			FT5402_RV_G_CHARGER_STATE		=	0;
			FT5402_RV_G_FILTERID_START		=	0;
			FT5402_FRAME_FILTER_EN		=	1;
			FT5402_FRAME_FILTER_SUB_MAX_TH		=	18;
			FT5402_FRAME_FILTER_ADD_MAX_TH		=	18;
			FT5402_FRAME_FILTER_SKIP_START_FRAME =		2;
			FT5402_FRAME_FILTER_BAND_EN		=	1;
			FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
			FT5402_OTP_PARAM_ID		=	0;
			
			g_ft5402_tx_num = 26;
			g_ft5402_rx_num = 16;
			g_ft5402_gain= 31;
			g_ft5402_voltage = 5;
			g_ft5402_scanselect= 6;
			g_ft5402_tx_offset = 0;
			
			unsigned char g_ft5402_tx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
			unsigned char g_ft5402_tx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
			unsigned char g_ft5402_rx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
			unsigned char g_ft5402_rx_offset_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU[] = {67,67,83,68,52,67,68,67};
			unsigned char g_ft5402_rx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};

			g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU), GFP_KERNEL);
			memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU, sizeof(g_ft5402_tx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU));
			
			g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU), GFP_KERNEL);
			memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU, sizeof(g_ft5402_tx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU));
			
			g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU, sizeof(g_ft5402_rx_order_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU));
			
			g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU, sizeof(g_ft5402_rx_offset_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU));
			
			g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU), GFP_KERNEL);
			memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU, sizeof(g_ft5402_rx_cap_used_K1001LC_YIPUDA_DPT_JKY502_0502000EU));
			break;

#endif

#if 1
			case 416:
				printk("project:D100lC\n");
				printk("client:ming  zhi\n");
				printk("Charge:BY120502000\n");
				printk("LCD:1024 600\n");
				printk("TP:YUN TIAN GE\n");
				printk("Author:ZENG QING MING\n");
				printk("Date:2014-04-17\n");
				
				FT5402_START_RX 	=	0;
				FT5402_ADC_TARGET		=	8500;
				FT5402_KX		=	151;
				FT5402_KY		=	160; 
				FT5402_RESOLUTION_X 	=	600;
				FT5402_RESOLUTION_Y 	=	1024;
				FT5402_LEMDA_X		=	5; 
				FT5402_LEMDA_Y		=	4; 
				FT5402_PWMODE_CTRL	=		1;
				FT5402_POINTS_SUPPORTED 	=	5;
				FT5402_DRAW_LINE_TH 	=	150;
				FT5402_FACE_DETECT_MODE 	=	0;
				FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
				FT5402_FACE_DETECT_PRE_VALUE	=		20;
				FT5402_FACE_DETECT_NUM		=	10;
				FT5402_THGROUP	=		22;
				FT5402_THPEAK		=	80;
				FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
				FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
				FT5402_MIN_DELTA_X	=		5;
				FT5402_MIN_DELTA_Y		=	5;
				FT5402_MIN_DELTA_STEP	=		2;
				FT5402_ESD_DIFF_VAL 	=	20;
				FT5402_ESD_NEGTIVE		=	-50;
				FT5402_ESD_FILTER_FRAME 	=	10;
				FT5402_MAX_TOUCH_VALUE		=	600;
				FT5402_CUSTOMER_ID		=	121;
				FT5402_IO_LEVEL_SELECT	=		0;
				FT5402_DIRECTION	=	1;
				FT5402_POINTID_DELAY_COUNT		=	6;
				FT5402_LIFTUP_FILTER_MACRO		=	0;
				FT5402_POINTS_STABLE_MACRO		=	1;
				FT5402_ESD_NOISE_MACRO		=	0;
				FT5402_RV_G_PERIOD_ACTIVE		=	14;
				FT5402_DIFFDATA_HANDLE		=	0;
				FT5402_MIN_WATER_VAL	=	-50;
				FT5402_MAX_NOISE_VAL	=	10;
				FT5402_WATER_HANDLE_START_RX =		0;
				FT5402_WATER_HANDLE_START_TX =		0;
				FT5402_HOST_NUMBER_SUPPORTED =		1;
				FT5402_RV_G_RAISE_THGROUP		=	28;
				FT5402_RV_G_CHARGER_STATE		=	0;
				FT5402_RV_G_FILTERID_START		=	0;
				FT5402_FRAME_FILTER_EN		=	1;
				FT5402_FRAME_FILTER_SUB_MAX_TH		=	8; 
				FT5402_FRAME_FILTER_ADD_MAX_TH		=	8;
				FT5402_FRAME_FILTER_SKIP_START_FRAME =		6; 
				FT5402_FRAME_FILTER_BAND_EN 	=	1;
				FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
				//FT5402_POWER_NOISE_FILTER_PROCESS_EN	=1;
				//FT5402_POWER_NOISE_RX_PEAK_DIFF	=	90;
				//FT5402_POWER_NOISE_RX_NUM =	2;  
				FT5402_OTP_PARAM_ID 	=	0;
				
				g_ft5402_tx_num = 26;
				g_ft5402_rx_num = 16;
				g_ft5402_gain= 18;
				g_ft5402_voltage = 4;
				g_ft5402_scanselect= 7;
				g_ft5402_tx_offset = 0;
				
				unsigned char g_ft5402_tx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
	
				unsigned char g_ft5402_tx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000[] = {60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,95};
	
				unsigned char g_ft5402_rx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
				unsigned char g_ft5402_rx_offset_used_D1001C_MINGZHI_YUNTIANGE_BY120502000[] = {51,35,67,18,99,67,51,2};
	
				unsigned char g_ft5402_rx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000[] = {140,140,140,140,140,140,140,140,140,140,140,140,140,140,140,140};
				
				g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000), GFP_KERNEL);
				memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000, sizeof(g_ft5402_tx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000));
				
				g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000), GFP_KERNEL);
				memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000, sizeof(g_ft5402_tx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000));
				
				g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000), GFP_KERNEL);
				memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000, sizeof(g_ft5402_rx_order_used_D1001C_MINGZHI_YUNTIANGE_BY120502000));
				
				g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D1001C_MINGZHI_YUNTIANGE_BY120502000), GFP_KERNEL);
				memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D1001C_MINGZHI_YUNTIANGE_BY120502000, sizeof(g_ft5402_rx_offset_used_D1001C_MINGZHI_YUNTIANGE_BY120502000));
				
				g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000), GFP_KERNEL);
				memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000, sizeof(g_ft5402_rx_cap_used_D1001C_MINGZHI_YUNTIANGE_BY120502000));
				break;
		#endif


	case 81:
		printk("project:D81\n");
		printk("Charge:XHY\n");
		printk("LCD:800 480\n");
		printk("TP:CLGD PG\n");
		printk("Author:zhangshengxian\n");
		printk("Date:2014-02-20_10:41\n");
		
		FT5402_START_RX 	=	0;
		FT5402_ADC_TARGET		=	8500;
		FT5402_KX		=	121;
		FT5402_KY		=	124;
		FT5402_RESOLUTION_X 	=	480;
		FT5402_RESOLUTION_Y 	=	800;
		FT5402_LEMDA_X		=	5;
		FT5402_LEMDA_Y		=	7;
		FT5402_PWMODE_CTRL	=		1;
		FT5402_POINTS_SUPPORTED 	=	5;
		FT5402_DRAW_LINE_TH 	=	150;
		FT5402_FACE_DETECT_MODE 	=	0;
		FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
		FT5402_FACE_DETECT_PRE_VALUE	=		20;
		FT5402_FACE_DETECT_NUM		=	10;
		FT5402_THGROUP	=		25;
		FT5402_THPEAK		=	60;
		FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
		FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
		FT5402_MIN_DELTA_X	=		5;
		FT5402_MIN_DELTA_Y		=	5;
		FT5402_MIN_DELTA_STEP	=		2;
		FT5402_ESD_DIFF_VAL		=	20;
		FT5402_ESD_NEGTIVE		=	-50;
		FT5402_ESD_FILTER_FRAME		=	10;
		FT5402_MAX_TOUCH_VALUE		=	600;
		FT5402_CUSTOMER_ID		=	121;
		FT5402_IO_LEVEL_SELECT	=		0;
		FT5402_DIRECTION 	=	1;
		FT5402_POINTID_DELAY_COUNT		=	2;
		FT5402_LIFTUP_FILTER_MACRO		=	0;
		FT5402_POINTS_STABLE_MACRO		=	1;
		FT5402_ESD_NOISE_MACRO		=	0;
		FT5402_RV_G_PERIOD_ACTIVE		=	16;
		FT5402_DIFFDATA_HANDLE		=	0;
		FT5402_MIN_WATER_VAL 	=	-50;
		FT5402_MAX_NOISE_VAL 	=	10;
		FT5402_WATER_HANDLE_START_RX =		0;
		FT5402_WATER_HANDLE_START_TX =		0;
		FT5402_HOST_NUMBER_SUPPORTED =		0;
		FT5402_RV_G_RAISE_THGROUP		=	30;
		FT5402_RV_G_CHARGER_STATE		=	0;
		FT5402_RV_G_FILTERID_START		=	2;
		FT5402_FRAME_FILTER_EN		=	0;
		FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
		FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
		FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
		FT5402_FRAME_FILTER_BAND_EN		=	1;
		FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
		FT5402_OTP_PARAM_ID		=	0;
		
		g_ft5402_tx_num = 26;
		g_ft5402_rx_num = 16;
		g_ft5402_gain= 21;
		g_ft5402_voltage = 4;
		g_ft5402_scanselect= 5;
		g_ft5402_tx_offset = 0;
		
		unsigned char g_ft5402_tx_order_used_D81[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
		unsigned char g_ft5402_tx_cap_used_D81[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
		unsigned char g_ft5402_rx_order_used_D81[] = {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0};
		unsigned char g_ft5402_rx_offset_used_D81[] ={51,51,52,51,67,51,51,68};
		unsigned char g_ft5402_rx_cap_used_D81[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};

		g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D81), GFP_KERNEL);
		memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D81, sizeof(g_ft5402_tx_order_used_D81));
		
		g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D81), GFP_KERNEL);
		memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D81, sizeof(g_ft5402_tx_cap_used_D81));
		
		g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D81), GFP_KERNEL);
		memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D81, sizeof(g_ft5402_rx_order_used_D81));
		
		g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D81), GFP_KERNEL);
		memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D81, sizeof(g_ft5402_rx_offset_used_D81));
		
		g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D81), GFP_KERNEL);
		memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D81, sizeof(g_ft5402_rx_cap_used_D81));
		break;

#if 0			
	case 408:
		printk("project:D901HC\n");
		printk("Charge:SUN-0500200\n");
		printk("LCD:1024 600\n");
		printk("TP:QSD\n");
		printk("Author:zhangshengxian\n");
		printk("Date:2014-02-18_17:17\n");
		
		FT5402_START_RX 	=	0;
		FT5402_ADC_TARGET		=	8500;
		FT5402_KX		=	152;
		FT5402_KY		=	160;
		FT5402_RESOLUTION_X 	=	600;
		FT5402_RESOLUTION_Y 	=	1024;
		FT5402_LEMDA_X		=	7;
		FT5402_LEMDA_Y		=	5;
		FT5402_PWMODE_CTRL	=		1;
		FT5402_POINTS_SUPPORTED 	=	5;
		FT5402_DRAW_LINE_TH 	=	150;
		FT5402_FACE_DETECT_MODE 	=	0;
		FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
		FT5402_FACE_DETECT_PRE_VALUE	=		20;
		FT5402_FACE_DETECT_NUM		=	10;
		FT5402_THGROUP	=		18;
		FT5402_THPEAK		=	60;
		FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
		FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
		FT5402_MIN_DELTA_X	=		4;
		FT5402_MIN_DELTA_Y		=	4;
		FT5402_MIN_DELTA_STEP	=		2;
		FT5402_ESD_DIFF_VAL		=	20;
		FT5402_ESD_NEGTIVE		=	-50;
		FT5402_ESD_FILTER_FRAME		=	10;
		FT5402_MAX_TOUCH_VALUE		=	600;
		FT5402_CUSTOMER_ID		=	121;
		FT5402_IO_LEVEL_SELECT	=		0;
		FT5402_DIRECTION 	=	1;
		FT5402_POINTID_DELAY_COUNT		=	2;
		FT5402_LIFTUP_FILTER_MACRO		=	0;
		FT5402_POINTS_STABLE_MACRO		=	1;
		FT5402_ESD_NOISE_MACRO		=	0;
		FT5402_RV_G_PERIOD_ACTIVE		=	16;
		FT5402_DIFFDATA_HANDLE		=	0;
		FT5402_MIN_WATER_VAL 	=	-50;
		FT5402_MAX_NOISE_VAL 	=	10;
		FT5402_WATER_HANDLE_START_RX =		0;
		FT5402_WATER_HANDLE_START_TX =		0;
		FT5402_HOST_NUMBER_SUPPORTED =		1;
		FT5402_RV_G_RAISE_THGROUP		=	20;
		FT5402_RV_G_CHARGER_STATE		=	0;
		FT5402_RV_G_FILTERID_START		=	1;
		FT5402_FRAME_FILTER_EN		=	0;
		FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
		FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
		FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
		FT5402_FRAME_FILTER_BAND_EN		=	1;
		FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
		FT5402_POWER_NOISE_FILTER_PROCESS_EN	=	0;
		FT5402_POWER_NOISE_RX_PEAK_DIFF	=	90;
		FT5402_POWER_NOISE_RX_NUM	=	2;
		FT5402_OTP_PARAM_ID		=	0;
		
		g_ft5402_tx_num = 26;
		g_ft5402_rx_num = 16;
		g_ft5402_gain= 20;
		g_ft5402_voltage = 4;
		g_ft5402_scanselect= 3;
		g_ft5402_tx_offset = 0;
		
		unsigned char g_ft5402_tx_order_used_D901HC[] ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
		unsigned char g_ft5402_tx_cap_used_D901HC[] = {42,43,44,45,46,47,48,49,50,51,52,53,54,56,58,60,62,64,66,68,70,72,74,76,78,85};
		unsigned char g_ft5402_rx_order_used_D901HC[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		unsigned char g_ft5402_rx_offset_used_D901HC[] = {67,52,67,51,35,52,37,52};
		unsigned char g_ft5402_rx_cap_used_D901HC[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
		g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D901HC), GFP_KERNEL);
		memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D901HC, sizeof(g_ft5402_tx_order_used_D901HC));
		
		g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D901HC), GFP_KERNEL);
		memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D901HC, sizeof(g_ft5402_tx_cap_used_D901HC));
		
		g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D901HC), GFP_KERNEL);
		memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D901HC, sizeof(g_ft5402_rx_order_used_D901HC));
		
		g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D901HC), GFP_KERNEL);
		memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D901HC, sizeof(g_ft5402_rx_offset_used_D901HC));
		
		g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D901HC), GFP_KERNEL);
		memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D901HC, sizeof(g_ft5402_rx_cap_used_D901HC));
		break;		
#endif

#if 0
	case 9021:
		printk("project:D901HC\n");
		printk("clientele:liu hong\n");
		printk("Charge:xx\n");
		printk("LCD:\n");
		printk("TP:de pu te\n");
		printk("Author:ZENG QING MING\n");
		printk("Date:2014-03-5\n");
		FT5402_START_RX 	=	0;
		FT5402_ADC_TARGET		=	8500;
		FT5402_KX		=	152;
		FT5402_KY		=	160;
		FT5402_RESOLUTION_X 	=	600;
		FT5402_RESOLUTION_Y 	=	1024;
		FT5402_LEMDA_X		=	0; 
		FT5402_LEMDA_Y		=	0; 
		FT5402_PWMODE_CTRL	=		1;
		FT5402_POINTS_SUPPORTED 	=	5;
		FT5402_DRAW_LINE_TH 	=	150;
		FT5402_FACE_DETECT_MODE 	=	0;
		FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
		FT5402_FACE_DETECT_PRE_VALUE	=		20;
		FT5402_FACE_DETECT_NUM		=	10;
		FT5402_THGROUP	=		22;
		FT5402_THPEAK		=	80;
		FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
		FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
		FT5402_MIN_DELTA_X	=		8;
		FT5402_MIN_DELTA_Y		=	8;
		FT5402_MIN_DELTA_STEP	=		2;
		FT5402_ESD_DIFF_VAL		=	20;
		FT5402_ESD_NEGTIVE		=	-50;
		FT5402_ESD_FILTER_FRAME		=	10;
		FT5402_MAX_TOUCH_VALUE		=	600;
		FT5402_CUSTOMER_ID		=	121;
		FT5402_IO_LEVEL_SELECT	=		0;
		FT5402_DIRECTION 	=	1;
		FT5402_POINTID_DELAY_COUNT		=	6;
		FT5402_LIFTUP_FILTER_MACRO		=	0;
		FT5402_POINTS_STABLE_MACRO		=	1; 
		FT5402_ESD_NOISE_MACRO		=	0;
		FT5402_RV_G_PERIOD_ACTIVE		=	16;
		FT5402_DIFFDATA_HANDLE		=	0;
		FT5402_MIN_WATER_VAL 	=	-50;
		FT5402_MAX_NOISE_VAL 	=	10;
		FT5402_WATER_HANDLE_START_RX =		0;
		FT5402_WATER_HANDLE_START_TX =		0;
		FT5402_HOST_NUMBER_SUPPORTED =		0;
		FT5402_RV_G_RAISE_THGROUP		=	30;
		FT5402_RV_G_CHARGER_STATE		=	0;
		FT5402_RV_G_FILTERID_START		=	0;
		FT5402_FRAME_FILTER_EN		=	1;
		FT5402_FRAME_FILTER_SUB_MAX_TH		=	6;
		FT5402_FRAME_FILTER_ADD_MAX_TH		=	6;
		FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
		FT5402_FRAME_FILTER_BAND_EN		=	1;
		FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
		FT5402_OTP_PARAM_ID		=	0;
		
		g_ft5402_tx_num = 26;
		g_ft5402_rx_num = 16;
		g_ft5402_gain= 22;
		g_ft5402_voltage = 4; 
		g_ft5402_scanselect= 14;
		g_ft5402_tx_offset = 0;
		
		unsigned char g_ft5402_tx_order_used_D901C_YINGDEIER_QLT_DM050200[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
		//unsigned char g_ft5402_tx_cap_used_D901C_YINGDEIER_QLT_DM050200[] = {36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,60,62,64,66,72,78,84,90,96,102,108};

        unsigned char g_ft5402_tx_cap_used_D901C_YINGDEIER_QLT_DM050200[] = {60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,90};
		
		unsigned char g_ft5402_rx_order_used_D901C_YINGDEIER_QLT_DM050200[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		unsigned char g_ft5402_rx_offset_used_D901C_YINGDEIER_QLT_DM050200[] = {67,67,83,68,52,67,68,67};
		unsigned char g_ft5402_rx_cap_used_D901C_YINGDEIER_QLT_DM050200[] = {120,120,120,120,120,120,120,120,120,120,120,120,120,120,120,120};

		g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used_D901C_YINGDEIER_QLT_DM050200), GFP_KERNEL);
		memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used_D901C_YINGDEIER_QLT_DM050200, sizeof(g_ft5402_tx_order_used_D901C_YINGDEIER_QLT_DM050200));
		
		g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used_D901C_YINGDEIER_QLT_DM050200), GFP_KERNEL);
		memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used_D901C_YINGDEIER_QLT_DM050200, sizeof(g_ft5402_tx_cap_used_D901C_YINGDEIER_QLT_DM050200));
		
		g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used_D901C_YINGDEIER_QLT_DM050200), GFP_KERNEL);
		memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used_D901C_YINGDEIER_QLT_DM050200, sizeof(g_ft5402_rx_order_used_D901C_YINGDEIER_QLT_DM050200));
		
		g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used_D901C_YINGDEIER_QLT_DM050200), GFP_KERNEL);
		memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used_D901C_YINGDEIER_QLT_DM050200, sizeof(g_ft5402_rx_offset_used_D901C_YINGDEIER_QLT_DM050200));
		
		g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used_D901C_YINGDEIER_QLT_DM050200), GFP_KERNEL);
		memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used_D901C_YINGDEIER_QLT_DM050200, sizeof(g_ft5402_rx_cap_used_D901C_YINGDEIER_QLT_DM050200));
		break;
#endif



	default:
		printk("project:D901HC\n");
		printk("Charge:XHY\n");
		printk("LCD:1024 600\n");
		printk("TP:INET BOM\n");
		printk("Author:mbgalex\n");
		printk("Date:2013-11-28_10:28\n");

		FT5402_START_RX 	=	0;
		FT5402_ADC_TARGET		=	8500;
		FT5402_KX		=	152;
		FT5402_KY		=	160;
		FT5402_RESOLUTION_X 	=	600;
		FT5402_RESOLUTION_Y 	=	1024;
		FT5402_LEMDA_X		=	7;
		FT5402_LEMDA_Y		=	5;
		FT5402_PWMODE_CTRL	=		1;
		FT5402_POINTS_SUPPORTED 	=	5;
		FT5402_DRAW_LINE_TH 	=	150;
		FT5402_FACE_DETECT_MODE 	=	0;
		FT5402_FACE_DETECT_STATISTICS_TX_NUM	=		3;
		FT5402_FACE_DETECT_PRE_VALUE	=		20;
		FT5402_FACE_DETECT_NUM		=	10;
		FT5402_THGROUP	=		24;
		FT5402_THPEAK		=	60;
		FT5402_BIGAREA_PEAK_VALUE_MIN		=	100;
		FT5402_BIGAREA_DIFF_VALUE_OVER_NUM		=	50;
		FT5402_MIN_DELTA_X	=		2;
		FT5402_MIN_DELTA_Y		=	2;
		FT5402_MIN_DELTA_STEP	=		2;
		FT5402_ESD_DIFF_VAL		=	20;
		FT5402_ESD_NEGTIVE		=	-50;
		FT5402_ESD_FILTER_FRAME		=	10;
		FT5402_MAX_TOUCH_VALUE		=	600;
		FT5402_CUSTOMER_ID		=	121;
		FT5402_IO_LEVEL_SELECT	=		0;
		FT5402_DIRECTION 	=	1;
		FT5402_POINTID_DELAY_COUNT		=	4;
		FT5402_LIFTUP_FILTER_MACRO		=	0;
		FT5402_POINTS_STABLE_MACRO		=	1;
		FT5402_ESD_NOISE_MACRO		=	0;
		FT5402_RV_G_PERIOD_ACTIVE		=	16;
		FT5402_DIFFDATA_HANDLE		=	0;
		FT5402_MIN_WATER_VAL 	=	-50;
		FT5402_MAX_NOISE_VAL 	=	10;
		FT5402_WATER_HANDLE_START_RX =		0;
		FT5402_WATER_HANDLE_START_TX =		0;
		FT5402_HOST_NUMBER_SUPPORTED =		0;
		FT5402_RV_G_RAISE_THGROUP		=	30;
		FT5402_RV_G_CHARGER_STATE		=	0;
		FT5402_RV_G_FILTERID_START		=	2;
		FT5402_FRAME_FILTER_EN		=	0;
		FT5402_FRAME_FILTER_SUB_MAX_TH		=	2;
		FT5402_FRAME_FILTER_ADD_MAX_TH		=	2;
		FT5402_FRAME_FILTER_SKIP_START_FRAME =		6;
		FT5402_FRAME_FILTER_BAND_EN		=	1;
		FT5402_FRAME_FILTER_BAND_WIDTH		=	128;
		FT5402_OTP_PARAM_ID		=	0;
		
		g_ft5402_tx_num = 26;
		g_ft5402_rx_num = 16;
		g_ft5402_gain= 20;
		g_ft5402_voltage = 4;
		g_ft5402_scanselect= 8;
		g_ft5402_tx_offset = 0;
		
		unsigned char g_ft5402_tx_order_used[] ={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25};
		unsigned char g_ft5402_tx_cap_used[] = {50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50};
		unsigned char g_ft5402_rx_order_used[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
		unsigned char g_ft5402_rx_offset_used[] = {52,68,67,52,68,68,68,68};
		unsigned char g_ft5402_rx_cap_used[] = {100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
		g_ft5402_tx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_order_used), GFP_KERNEL);
		memcpy(g_ft5402_tx_order, g_ft5402_tx_order_used, sizeof(g_ft5402_tx_order_used));
		
		g_ft5402_tx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_tx_cap_used), GFP_KERNEL);
		memcpy(g_ft5402_tx_cap, g_ft5402_tx_cap_used, sizeof(g_ft5402_tx_cap_used));
		
		g_ft5402_rx_order = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_order_used), GFP_KERNEL);
		memcpy(g_ft5402_rx_order, g_ft5402_rx_order_used, sizeof(g_ft5402_rx_order_used));
		
		g_ft5402_rx_offset = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_offset_used), GFP_KERNEL);
		memcpy(g_ft5402_rx_offset, g_ft5402_rx_offset_used, sizeof(g_ft5402_rx_offset_used));
		
		g_ft5402_rx_cap = (unsigned char *)kzalloc(sizeof(g_ft5402_rx_cap_used), GFP_KERNEL);
		memcpy(g_ft5402_rx_cap, g_ft5402_rx_cap_used, sizeof(g_ft5402_rx_cap_used));
		break;


			
	}

	printk("+++++++++++++++++++++++++++++++++++++++++\n");

	//g_param_ft5402
	g_param_ft5402.ft5402_KX=FT5402_KX,
	g_param_ft5402.ft5402_KY=FT5402_KY;
	g_param_ft5402.ft5402_LEMDA_X=FT5402_LEMDA_X;
	g_param_ft5402.ft5402_LEMDA_Y=FT5402_LEMDA_Y;
	g_param_ft5402.ft5402_RESOLUTION_X=FT5402_RESOLUTION_X;
	g_param_ft5402.ft5402_RESOLUTION_Y=FT5402_RESOLUTION_Y;
	g_param_ft5402.ft5402_DIRECTION=FT5402_DIRECTION;
	g_param_ft5402.ft5402_FACE_DETECT_PRE_VALUE=FT5402_FACE_DETECT_PRE_VALUE;
	g_param_ft5402.ft5402_FACE_DETECT_NUM=FT5402_FACE_DETECT_NUM;

	g_param_ft5402.ft5402_BIGAREA_PEAK_VALUE_MIN=FT5402_BIGAREA_PEAK_VALUE_MIN;
	g_param_ft5402.ft5402_BIGAREA_DIFF_VALUE_OVER_NUM=	FT5402_BIGAREA_DIFF_VALUE_OVER_NUM;
	g_param_ft5402.ft5402_CUSTOMER_ID=FT5402_CUSTOMER_ID;
	g_param_ft5402.ft5402_PERIOD_ACTIVE=FT5402_RV_G_PERIOD_ACTIVE;
	g_param_ft5402.ft5402_FACE_DETECT_STATISTICS_TX_NUM=FT5402_FACE_DETECT_STATISTICS_TX_NUM;

	g_param_ft5402.ft5402_THGROUP=FT5402_THGROUP;
	g_param_ft5402.ft5402_THPEAK=FT5402_THPEAK;
	g_param_ft5402.ft5402_FACE_DETECT_MODE=FT5402_FACE_DETECT_MODE;
	g_param_ft5402.ft5402_MAX_TOUCH_VALUE=FT5402_MAX_TOUCH_VALUE;

	g_param_ft5402.ft5402_PWMODE_CTRL=FT5402_PWMODE_CTRL;
	g_param_ft5402.ft5402_DRAW_LINE_TH=FT5402_DRAW_LINE_TH;
	g_param_ft5402.ft5402_POINTS_SUPPORTED=FT5402_POINTS_SUPPORTED;
			
	g_param_ft5402.ft5402_START_RX=FT5402_START_RX;
	g_param_ft5402.ft5402_ADC_TARGET=FT5402_ADC_TARGET;
	g_param_ft5402.ft5402_ESD_FILTER_FRAME=FT5402_ESD_FILTER_FRAME;

	g_param_ft5402.ft5402_POINTS_STABLE_MACRO=FT5402_POINTS_STABLE_MACRO;
	g_param_ft5402.ft5402_MIN_DELTA_X=FT5402_MIN_DELTA_X;
	g_param_ft5402.ft5402_MIN_DELTA_Y=FT5402_MIN_DELTA_Y;
	g_param_ft5402.ft5402_MIN_DELTA_STEP=FT5402_MIN_DELTA_STEP;
		
	g_param_ft5402.ft5402_ESD_NOISE_MACRO=FT5402_ESD_NOISE_MACRO;
	g_param_ft5402.ft5402_ESD_DIFF_VAL=FT5402_ESD_DIFF_VAL;
	g_param_ft5402.ft5402_ESD_NEGTIVE=FT5402_ESD_NEGTIVE;	//negtive
	g_param_ft5402.ft5402_ESD_FILTER_FRAMES=FT5402_ESD_FILTER_FRAME;

	g_param_ft5402.ft5402_IO_LEVEL_SELECT=FT5402_IO_LEVEL_SELECT;

	g_param_ft5402.ft5402_POINTID_DELAY_COUNT=FT5402_POINTID_DELAY_COUNT;

	g_param_ft5402.ft5402_LIFTUP_FILTER_MACRO=FT5402_LIFTUP_FILTER_MACRO;

	g_param_ft5402.ft5402_DIFF_HANDLE_MACRO=FT5402_DIFFDATA_HANDLE;
	g_param_ft5402.ft5402_MIN_WATER=FT5402_MIN_WATER_VAL;	//negtive
	g_param_ft5402.ft5402_MAX_NOISE=FT5402_MAX_NOISE_VAL;
	g_param_ft5402.ft5402_WATER_START_RX=FT5402_WATER_HANDLE_START_RX;
	g_param_ft5402.ft5402_WATER_START_TX=FT5402_WATER_HANDLE_START_TX;

	g_param_ft5402.ft5402_HOST_NUMBER_SUPPORTED_MACRO=FT5402_HOST_NUMBER_SUPPORTED;
	g_param_ft5402.ft5402_RAISE_THGROUP=FT5402_RV_G_RAISE_THGROUP;
	g_param_ft5402.ft5402_CHARGER_STATE=FT5402_RV_G_CHARGER_STATE;

	g_param_ft5402.ft5402_FILTERID_START=FT5402_RV_G_FILTERID_START;

	g_param_ft5402.ft5402_FRAME_FILTER_EN_MACRO=FT5402_FRAME_FILTER_EN;
	g_param_ft5402.ft5402_FRAME_FILTER_SUB_MAX_TH=FT5402_FRAME_FILTER_SUB_MAX_TH;
	g_param_ft5402.ft5402_FRAME_FILTER_ADD_MAX_TH=FT5402_FRAME_FILTER_ADD_MAX_TH;
	g_param_ft5402.ft5402_FRAME_FILTER_SKIP_START_FRAME=FT5402_FRAME_FILTER_SKIP_START_FRAME;
	g_param_ft5402.ft5402_FRAME_FILTER_BAND_EN=FT5402_FRAME_FILTER_BAND_EN;
	g_param_ft5402.ft5402_FRAME_FILTER_BAND_WIDTH=FT5402_FRAME_FILTER_BAND_WIDTH;
	//mbg --

	
	//mbg --
	
	//mbg ++ 20131120
	g_param_ft5402.ft5402_POWER_NOISE_FILTER_PROCESS_EN=FT5402_POWER_NOISE_FILTER_PROCESS_EN;
	g_param_ft5402.ft5402_POWER_NOISE_RX_PEAK_DIFF=FT5402_POWER_NOISE_RX_PEAK_DIFF;
	g_param_ft5402.ft5402_POWER_NOISE_RX_NUM=FT5402_POWER_NOISE_RX_NUM;
	//mbg --
	
	/*enter factory mode*/
	err = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE, FT5402_FACTORYMODE_VALUE);
	if (err < 0) {
		dev_err(&client->dev, "%s:enter factory mode failed.\n", __func__);
		goto RETURN_WORK;
	}
	
	for (i = 0; i < g_ft5402_tx_num; i++) {
		if (g_ft5402_tx_order[i] != 0xFF) {
			/*set tx order*/
			err = ft5402_set_tx_order(client, i, g_ft5402_tx_order[i]);
			if (err < 0) {
				dev_err(&client->dev, "%s:could not set tx%d order.\n",
						__func__, i);
				goto RETURN_WORK;
			}
		}
		/*set tx cap*/
		err = ft5402_set_tx_cap(client, i, g_ft5402_tx_cap[i]);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not set tx%d cap.\n",
					__func__, i);
			goto RETURN_WORK;
		}
	}
	/*set tx offset*/
	err = ft5402_set_tx_offset(client, 0, g_ft5402_tx_offset);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set tx 0 offset.\n",
				__func__);
		goto RETURN_WORK;
	}

	/*set rx offset and cap*/
	for (i = 0; i < g_ft5402_rx_num; i++) {
		/*set rx order*/
		err = ft5402_set_rx_order(client, i, g_ft5402_rx_order[i]);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not set rx%d order.\n",
					__func__, i);
			goto RETURN_WORK;
		}
		/*set rx cap*/
		err = ft5402_set_rx_cap(client, i, g_ft5402_rx_cap[i]);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not set rx%d cap.\n",
					__func__, i);
			goto RETURN_WORK;
		}
	}
	for (i = 0; i < g_ft5402_rx_num/2; i++) {
		err = ft5402_set_rx_offset(client, i*2, g_ft5402_rx_offset[i]>>4);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not set rx offset.\n",
				__func__);
			goto RETURN_WORK;
		}
		err = ft5402_set_rx_offset(client, i*2+1, g_ft5402_rx_offset[i]&0x0F);
		if (err < 0) {
			dev_err(&client->dev, "%s:could not set rx offset.\n",
				__func__);
			goto RETURN_WORK;
		}
	}

	/*set scan select*/
	err = ft5402_set_scan_select(client, g_ft5402_scanselect);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set scan select.\n",
			__func__);
		goto RETURN_WORK;
	}
	
	/*set tx number*/
	err = ft5402_set_tx_num(client, g_ft5402_tx_num);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set tx num.\n",
			__func__);
		goto RETURN_WORK;
	}
	/*set rx number*/
	err = ft5402_set_rx_num(client, g_ft5402_rx_num);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set rx num.\n",
			__func__);
		goto RETURN_WORK;
	}
	
	/*set gain*/
	err = ft5402_set_gain(client, g_ft5402_gain);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set gain.\n",
			__func__);
		goto RETURN_WORK;
	}
	/*set voltage*/
	err = ft5402_set_vol(client, g_ft5402_voltage);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set voltage.\n",
			__func__);
		goto RETURN_WORK;
	}

	err = ft5402_write_reg(client, FT5402_REG_ADC_TARGET_HIGH,
			g_param_ft5402.ft5402_ADC_TARGET>>8);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ADC_TARGET_HIGH failed.\n", __func__);
		return err;
	}
	err = ft5402_write_reg(client, FT5402_REG_ADC_TARGET_LOW,
			g_param_ft5402.ft5402_ADC_TARGET);
	if (err < 0) {
		dev_err(&client->dev, "%s:write ADC_TARGET_LOW failed.\n", __func__);
		return err;
	}

RETURN_WORK:	
	/*enter work mode*/
	err = ft5402_write_reg(client, FT5402_REG_DEVICE_MODE, FT5402_WORKMODE_VALUE);
	if (err < 0) {
		dev_err(&client->dev, "%s:enter work mode failed.\n", __func__);
		goto ERR_EXIT;
	}

	/*set resolution*/
	err = ft5402_set_Resolution(client, g_param_ft5402.ft5402_RESOLUTION_X,
				g_param_ft5402.ft5402_RESOLUTION_Y);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set resolution.\n",
			__func__);
		goto ERR_EXIT;
	}

	/*set face detect statistics tx num*/
	err = ft5402_set_face_detect_statistics_tx_num(client,
			g_param_ft5402.ft5402_FACE_DETECT_STATISTICS_TX_NUM);
	if (err < 0) {
		dev_err(&client->dev,
				"%s:could not set face detect statistics tx num.\n",
				__func__);
		goto ERR_EXIT;
	}
	/*set face detect pre value*/
	err = ft5402_set_face_detect_pre_value(client,
			g_param_ft5402.ft5402_FACE_DETECT_PRE_VALUE);
	if (err < 0) {
		dev_err(&client->dev,
				"%s:could not set face detect pre value.\n",
				__func__);
		goto ERR_EXIT;
	}
	/*set face detect num*/
	err = ft5402_set_face_detect_num(client,
			g_param_ft5402.ft5402_FACE_DETECT_NUM);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set face detect num.\n",
			__func__);
		goto ERR_EXIT;
	}
	
	/*set min peak value*/
	err = ft5402_set_peak_value_min(client,
			g_param_ft5402.ft5402_BIGAREA_PEAK_VALUE_MIN);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set min peak value.\n",
			__func__);
		goto ERR_EXIT;
	}
	/*set diff value over num*/
	err = ft5402_set_diff_value_over_num(client,
			g_param_ft5402.ft5402_BIGAREA_DIFF_VALUE_OVER_NUM);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set diff value over num.\n",
			__func__);
		goto ERR_EXIT;
	}
	/*set customer id*/
	err = ft5402_set_customer_id(client,
			g_param_ft5402.ft5402_CUSTOMER_ID);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set customer id.\n",
			__func__);
		goto ERR_EXIT;
	}
	/*set kx*/
	err = ft5402_set_kx(client, g_param_ft5402.ft5402_KX);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set kx.\n",
			__func__);
		goto ERR_EXIT;
	}
	/*set ky*/
	err = ft5402_set_ky(client, g_param_ft5402.ft5402_KY);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set ky.\n",
			__func__);
		goto ERR_EXIT;
	}
	/*set lemda x*/
	err = ft5402_set_lemda_x(client,
			g_param_ft5402.ft5402_LEMDA_X);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set lemda x.\n",
			__func__);
		goto ERR_EXIT;
	}
	/*set lemda y*/
	err = ft5402_set_lemda_y(client,
			g_param_ft5402.ft5402_LEMDA_Y);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set lemda y.\n",
			__func__);
		goto ERR_EXIT;
	}
	/*set pos x*/
	err = ft5402_set_pos_x(client, g_param_ft5402.ft5402_DIRECTION);
	if (err < 0) {
		dev_err(&client->dev, "%s:could not set pos x.\n",
			__func__);
		goto ERR_EXIT;
	}

	err = ft5402_set_other_param(client);
	
ERR_EXIT:
	return err;
}
EXPORT_SYMBOL_GPL(ft5402_Init_IC_Param);


char dst[512];
static char * ft5402_sub_str(char * src, int n)
{
	char *p = src;
	int i;
	int m = 0;
	int len = strlen(src);

	while (n >= 1 && m <= len) {
		i = 0;
		dst[10] = ' ';
		n--;
		while ( *p != ',' && *p != ' ') {
			dst[i++] = *(p++);
			m++;
			if (i >= len)
				break;
		}
		dst[i++] = '\0';
		p++;
	}
	return dst;
}
static int ft5402_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s%s", FT5402_INI_FILEPATH, config_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadInIData(char *config_name,
			      char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5402_INI_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}

int ft5402_Get_Param_From_Ini(char *config_name)
{
	char key[64];
	char value[512];
	char section[64];
	int i = 0;//,ret=0;
	int j = 0;
	char *filedata = NULL;
	unsigned char legal_byte1 = 0x00;
	unsigned char legal_byte2 = 0x00;

	int inisize = ft5402_GetInISize(config_name);
	
	if (inisize <= 0) {
		pr_err("%s ERROR:Get firmware size failed\n",
					__func__);
		return -EIO;
	}

	filedata = kmalloc(inisize + 1, GFP_ATOMIC);
		
	if (ft5x0x_ReadInIData(config_name, filedata)) {
		pr_err("%s() - ERROR: request_firmware failed\n",
					__func__);
		kfree(filedata);
		return -EIO;
	}

	/*check ini  if  it is illegal*/
	sprintf(section, "%s", FT5402_APP_LEGAL);
	sprintf(key, "%s", FT5402_APP_LEGAL_BYTE_1_STR);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	legal_byte1 = atoi(value);
	DBG("legal_byte1=%s\n", value);
	sprintf(key, "%s", FT5402_APP_LEGAL_BYTE_2_STR);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	legal_byte2 = atoi(value);
	DBG("lega2_byte1=%s\n", value);
	if(FT5402_APP_LEGAL_BYTE_1_VALUE == legal_byte1 &&
		FT5402_APP_LEGAL_BYTE_2_VALUE == legal_byte2)
		DBG("the ini file is valid\n");
	else {
		pr_err("[FTS]-----the ini file is invalid!please check it.\n");
		goto ERROR_RETURN;
	}

	/*get ini param*/
	sprintf(section, "%s", FT5402_APP_NAME);
		
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_KX = atoi(value);
	DBG("ft5402_KX=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_KY = atoi(value);
	DBG("ft5402_KY=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_LEMDA_X = atoi(value);
	DBG("ft5402_LEMDA_X=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_LEMDA_Y = atoi(value);
	DBG("ft5402_LEMDA_Y=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_RESOLUTION_X = atoi(value);
	DBG("ft5402_RESOLUTION_X=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_RESOLUTION_Y = atoi(value);
	DBG("ft5402_RESOLUTION_Y=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_DIRECTION= atoi(value);
	DBG("ft5402_DIRECTION=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FACE_DETECT_PRE_VALUE = atoi(value);
	DBG("ft5402_FACE_DETECT_PRE_VALUE=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FACE_DETECT_NUM = atoi(value);
	DBG("ft5402_FACE_DETECT_NUM=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_BIGAREA_PEAK_VALUE_MIN = atoi(value);/*The min value to be decided as the big point*/
	DBG("ft5402_BIGAREA_PEAK_VALUE_MIN=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_BIGAREA_DIFF_VALUE_OVER_NUM = atoi(value);/*The min big points of the big area*/
	DBG("ft5402_BIGAREA_DIFF_VALUE_OVER_NUM=%s\n", value);
	
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_CUSTOMER_ID = atoi(value);
	DBG("ft5402_CUSTOM_ID=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_PERIOD_ACTIVE = atoi(value);
	DBG("ft5402_PERIOD_ACTIVE=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FACE_DETECT_STATISTICS_TX_NUM = atoi(value);
	DBG("ft5402_FACE_DETECT_STATISTICS_TX_NUM=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_THGROUP = atoi(value);
	DBG("ft5402_THGROUP=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_THPEAK = atoi(value);
	DBG("ft5402_THPEAK=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FACE_DETECT_MODE = atoi(value);
	DBG("ft5402_FACE_DETECT_MODE=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_MAX_TOUCH_VALUE = atoi(value);
	DBG("ft5402_MAX_TOUCH_VALUE=%s\n", value);

	
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_PWMODE_CTRL= atoi(value);
	DBG("ft5402_PWMODE_CTRL=%s\n", value);



	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;

	i++;
	g_param_ft5402.ft5402_DRAW_LINE_TH = atoi(value);
	DBG("ft5402_DRAW_LINE_TH=%s\n", value);


	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_POINTS_SUPPORTED= atoi(value);
	DBG("ft5402_POINTS_SUPPORTED=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_START_RX = atoi(value);
	DBG("ft5402_START_RX=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	
	g_param_ft5402.ft5402_ADC_TARGET = atoi(value);
	DBG("ft5402_ADC_TARGET=%s\n", value);


	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	
	g_param_ft5402.ft5402_ESD_FILTER_FRAME = atoi(value);
	DBG("ft5402_ESD_FILTER_FRAME=%s\n", value);

/*********************************************************************/	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_ft5402_tx_num = atoi(value);
	DBG("ft5402_tx_num=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_ft5402_rx_num = atoi(value);
	DBG("ft5402_rx_num=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_ft5402_gain = atoi(value);
	DBG("ft5402_gain=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_ft5402_voltage = atoi(value);
	DBG("ft5402_voltage=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_ft5402_scanselect = atoi(value);
	DBG("ft5402_scanselect=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	for(j = 0; j < g_ft5402_tx_num; j++)
	{
		char * psrc = value;
		g_ft5402_tx_order[j] = atoi(ft5402_sub_str(psrc, j+1));
	}
	DBG("ft5402_tx_order=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_ft5402_tx_offset = atoi(value);
	DBG("ft5402_tx_offset=%s\n", value);
	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	for(j = 0; j < g_ft5402_tx_num; j++)
	{
		char * psrc = value;
		g_ft5402_tx_cap[j] = atoi(ft5402_sub_str(psrc, j+1));
	}
	DBG("ft5402_tx_cap=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	for(j = 0; j < g_ft5402_rx_num; j++)
	{
		char * psrc = value;
		g_ft5402_rx_order[j] = atoi(ft5402_sub_str(psrc, j+1));
	}
	DBG("ft5402_rx_order=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	for(j = 0; j < g_ft5402_rx_num/2; j++)
	{
		char * psrc = value;
		g_ft5402_rx_offset[j] = atoi(ft5402_sub_str(psrc, j+1));
	}
	DBG("ft5402_rx_offset=%s\n", value);

	
	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	for(j = 0; j < g_ft5402_rx_num; j++)
	{
		char * psrc = value;
		g_ft5402_rx_cap[j] = atoi(ft5402_sub_str(psrc, j+1));
	}
	DBG("ft5402_rx_cap=%s\n", value);


	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_POINTS_STABLE_MACRO = atoi(value);
	DBG("ft5402_POINTS_STABLE_MACRO=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_MIN_DELTA_X = atoi(value);
	DBG("ft5402_MIN_DELTA_X=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_MIN_DELTA_Y = atoi(value);
	DBG("ft5402_MIN_DELTA_Y=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_MIN_DELTA_STEP = atoi(value);
	DBG("ft5402_MIN_DELTA_STEP=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_ESD_NOISE_MACRO = atoi(value);
	DBG("ft5402_ESD_NOISE_MACRO=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_ESD_DIFF_VAL = atoi(value);
	DBG("ft5402_ESD_DIFF_VAL=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_ESD_NEGTIVE = atoi(value);
	DBG("ft5402_ESD_NEGTIVE=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_ESD_FILTER_FRAMES = atoi(value);
	DBG("ft5402_ESD_FILTER_FRAMES=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_IO_LEVEL_SELECT = atoi(value);
	DBG("ft5402_IO_LEVEL_SELECT=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_POINTID_DELAY_COUNT = atoi(value);
	DBG("ft5402_POINTID_DELAY_COUNT=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_LIFTUP_FILTER_MACRO = atoi(value);
	DBG("ft5402_LIFTUP_FILTER_MACRO=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_DIFF_HANDLE_MACRO = atoi(value);
	DBG("ft5402_DIFF_HANDLE_MACRO=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_MIN_WATER = atoi(value);
	DBG("ft5402_MIN_WATER=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_MAX_NOISE = atoi(value);
	DBG("ft5402_MAX_NOISE=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_WATER_START_RX = atoi(value);
	DBG("ft5402_WATER_START_RX=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_WATER_START_TX = atoi(value);
	DBG("ft5402_WATER_START_TX=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_HOST_NUMBER_SUPPORTED_MACRO = atoi(value);
	DBG("ft5402_HOST_NUMBER_SUPPORTED_MACRO=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_RAISE_THGROUP = atoi(value);
	DBG("ft5402_RAISE_THGROUP=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_CHARGER_STATE = atoi(value);
	DBG("ft5402_CHARGER_STATE=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FILTERID_START = atoi(value);
	DBG("ft5402_FILTERID_START=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FRAME_FILTER_EN_MACRO = atoi(value);
	DBG("ft5402_FRAME_FILTER_EN_MACRO=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FRAME_FILTER_SUB_MAX_TH = atoi(value);
	DBG("ft5402_FRAME_FILTER_SUB_MAX_TH=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FRAME_FILTER_ADD_MAX_TH = atoi(value);
	DBG("ft5402_FRAME_FILTER_ADD_MAX_TH=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FRAME_FILTER_SKIP_START_FRAME = atoi(value);
	DBG("ft5402_FRAME_FILTER_SKIP_START_FRAME=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FRAME_FILTER_BAND_EN = atoi(value);
	DBG("ft5402_FRAME_FILTER_BAND_EN=%s\n", value);

	sprintf(key, "%s", String_Param_FT5402[i]);
	if (ini_get_key(filedata,section,key,value)<0)
		goto ERROR_RETURN;
	i++;
	g_param_ft5402.ft5402_FRAME_FILTER_BAND_WIDTH = atoi(value);
	DBG("ft5402_FRAME_FILTER_BAND_WIDTH=%s\n", value);

	//mbg ++ 20131120
		sprintf(key, "%s", String_Param_FT5402[i]);
		if (ini_get_key(filedata,section,key,value)<0)
			goto ERROR_RETURN;
		i++;
		g_param_ft5402.ft5402_POWER_NOISE_FILTER_PROCESS_EN = atoi(value);
		DBG("ft5402_POWER_NOISE_FILTER_PROCESS_EN=%s\n", value);
	
		sprintf(key, "%s", String_Param_FT5402[i]);
		if (ini_get_key(filedata,section,key,value)<0)
			goto ERROR_RETURN;
		i++;
		g_param_ft5402.ft5402_POWER_NOISE_RX_PEAK_DIFF = atoi(value);
		DBG("ft5402_POWER_NOISE_RX_PEAK_DIFF=%s\n", value);
	
		sprintf(key, "%s", String_Param_FT5402[i]);
		if (ini_get_key(filedata,section,key,value)<0)
			goto ERROR_RETURN;
		i++;
		g_param_ft5402.ft5402_POWER_NOISE_RX_NUM = atoi(value);
		DBG("ft5402_POWER_NOISE_RX_NUM=%s\n", value);
		//mbg -- 

	
	if (filedata) 
		kfree(filedata);
	return 0;
ERROR_RETURN:
	if (filedata) 
		kfree(filedata);
	return -1;
}
EXPORT_SYMBOL_GPL(ft5402_Get_Param_From_Ini);


