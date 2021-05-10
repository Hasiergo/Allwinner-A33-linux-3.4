#include <linux/delay.h>
//#include <mach/system.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <mach/sys_config.h>
//#include "inet_ctp.h"

//20140307 for tp light
static struct tp_light{
	struct gpio_config  gpio;
	struct timer_list timer;
	unsigned int timeout;	
}inet_tp_light;

static int have_light;

void tp_light_ctrl(int i)
{
	if(have_light == 0)
		return ;
	
	__gpio_set_value(inet_tp_light.gpio.gpio , i);
	if(i == 0)	//timmer off
	{
		
		mod_timer( &(inet_tp_light.timer),0);
	}
	else
		mod_timer( &(inet_tp_light.timer),jiffies + inet_tp_light.timeout/(1000/HZ));	
}

static int light_fetch_config(void)
{
	script_item_u   item;
	script_item_value_type_e   type;


	type = script_get_item("tp_light", "tp_light_gpio", &item);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		printk("light_fentch_config %s err\n", "tp_light_gpio");
		return 0;
	}
	inet_tp_light.gpio = item.gpio;

	if(0 != gpio_request(inet_tp_light.gpio.gpio, NULL)) {
		printk("tp_light_gpio gpio_request is failed\n");
		return 0;
	}

	if (0 != gpio_direction_output(inet_tp_light.gpio.gpio, 1)) {
		printk("tp_light_gpio gpio set err!");
		return 0;
	}

	printk("inet_tp_light.gpio.mul_sel[%#x]inet_tp_light.gpio.pull[%#x]inet_tp_light.gpio.data[%#x]\n",inet_tp_light.gpio.mul_sel,inet_tp_light.gpio.pull,inet_tp_light.gpio.data);
		
	type = script_get_item("tp_light", "timeout", &item);
	if(SCIRPT_ITEM_VALUE_TYPE_INT != type)
	{
		printk("light_fetch_config %s err\n", "timeout");
		return 0;
	}
	inet_tp_light.timeout = item.val;
	
	init_timer(&(inet_tp_light.timer));
	inet_tp_light.timer.function = tp_light_ctrl;
	inet_tp_light.timer.data = 0;	// turn off light
	inet_tp_light.timer.expires = jiffies + 15000/(1000/HZ);	
	add_timer(&(inet_tp_light.timer));
	
	__gpio_set_value(inet_tp_light.gpio.gpio , 1); //turn on light
	
	return 1;
}
int m_inet_ctpState=0;

static int inet_ctp_init(void)
{
	//Rocky@20131202+
	//enable usb drv_ctrl
	script_item_u   val;
	script_item_value_type_e  type;
	struct gpio_config usb_ctrl_gpio; 
	
	have_light=light_fetch_config();

		type = script_get_item("usbc0", "usb_drv_ctrl_gpio", &val);
	if (SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		printk("script_get_item ctp_wakeup err\n");
		goto script_get_item_err;
	}
	usb_ctrl_gpio = val.gpio;
	
	if(0 != gpio_request(usb_ctrl_gpio.gpio, NULL)) {
		printk(KERN_ERR "wakeup gpio_request is failed\n");
		goto script_get_item_err;
	}
	
	if (0 != gpio_direction_output(usb_ctrl_gpio.gpio, 1)) {
		printk(KERN_ERR "wakeup gpio set err!");
		goto script_get_item_err;
	}
	printk("Rocky@20131202@usb_ctrl\n");
	__gpio_set_value(usb_ctrl_gpio.gpio, 1);
	
script_get_item_err:
	//Rocky@20131202
	printk("inet_ctp_init\n");

	return 0;
}

static void inet_ctp_exit(void)
{
	printk("inet_ctp_exit");
}


module_init(inet_ctp_init);
module_exit(inet_ctp_exit);

EXPORT_SYMBOL(tp_light_ctrl);
EXPORT_SYMBOL(m_inet_ctpState);
MODULE_AUTHOR("<mbgalex@163.com>");
MODULE_DESCRIPTION("INET CTP CONTROL");
MODULE_LICENSE("Dual BSD/GPL");
