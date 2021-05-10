#include "camera_list.h"
#include "camera_detector.h"
//gongpiqiang+++ add camera
//#include "inet_camera_type.h" //no use now
//gongpiqiang+++ for camera info store
#include<linux/fs.h>
#include<linux/string.h>
#include<asm/uaccess.h>
#define FILE_DIR                "data/camera.info"
#define FILE_LENGTH             (1024)
#define STRING_LENGTH           (128)
#define YUV_30W_SENSOR "gc0308"
#define YUV_2M_SENSOR "gt2005"
#define YUV_5M_SENSOR "ov5640"
#define RAW_2M_SENSOR "gc2235"
#define RAW_5M_SENSOR "s5k4e1"


static char camera_type_buffer[32];

__u32 camera_type_index = 0;
int single_facing = 0;
int raw_sensor = 0;

static __camera_sensor_info all_sensor_info[] = {//add camera
//0.3M
	{"siv120d", 30, YUV},
	{"siv121d", 30, YUV},
	{"gc0308", 30, YUV},
	{"gc0309", 30, YUV},
	{"gc0311", 30, YUV},
	{"gc0312", 30, YUV},	
	{"gc0328", 30, YUV},
	{"gc0329", 30, YUV},
	{"sp0a19", 30, YUV},	
	{"sp0718", 30, YUV},
	{"sp0838", 30, YUV},
	{"bf3a03", 30, YUV},	
	{"bf3703", 30, YUV},
	{"hm0357", 30, YUV},		
//2M	
  {"hi253", 200, YUV},
	{"gt2005", 200, YUV},
	{"gc2035", 200, YUV},
	{"sp2518", 200, YUV},
	{"bf3920", 200, YUV},
	{"gc2155", 200, YUV},
	{"gc2145", 200, YUV},	
	{"sid130b", 200, YUV},
	{"sp2519", 200, YUV},		
//	{"gc2235", 200, RAW_2M},
//5M
	{"ov5640", 500, YUV},
	{"s5k4ec", 500, YUV},		
//	{"s5k4e1", 500, RAW_5M},
};

//gongpiqiang---
__camera_detector_t camera_detector[1];
extern  void camera_stby_on_sensor(__u32 list_index, __camera_info_t *camera_info);


__bool camera_sub_name_exist(char *main_name, char *sub_name);

//gongpiqiang+++ for camera info store
static int get_camera_type(char **camera_type);
static int camera_info_done = 0;
struct camera_detect_info{
        char type[STRING_LENGTH];
        char back_name[STRING_LENGTH];
        char front_name[STRING_LENGTH];
};
static struct camera_detect_info camera_info;
static int get_device_info(void)
{
        mm_segment_t old_fs;
        int ret;
     		struct file *filp = NULL;   
     		char src_string[FILE_LENGTH],tmp[FILE_LENGTH];
    		memset(&src_string, 0, sizeof(src_string));
        filp = filp_open(FILE_DIR,O_RDWR | O_CREAT, 0666);
        if(!filp || IS_ERR(filp)){
                printk("%s:open error ....IS(filp):%ld\n", __func__, IS_ERR(filp));
                return -1;
        } 
        
        old_fs = get_fs();
        set_fs(get_ds());
        filp->f_op->llseek(filp, 0, 0);
        ret = filp->f_op->read(filp, src_string, FILE_LENGTH, &filp->f_pos);
        
        if(ret <= 0){
                printk("%s:read erro or read null !\n", __func__);
        }
        
        set_fs(old_fs);
        filp_close(filp, NULL);

        printk("camera.info:%s\n",src_string);
        //解析camera.info文件的信息
        int i = 0, j = 0, k = 0, begin = 0;          
        while(src_string[i++]){
        				if(k == 0 && begin) {//type
        					if(src_string[i-1] == '\n')
        						camera_info.type[j] = '\0';
        					else
        						camera_info.type[j++] = src_string[i-1];
        				}
        				if(k == 1 && begin) {//back_name
        					if(src_string[i-1] == '\n')
        						camera_info.back_name[j] = '\0';
        				  else
        				  	camera_info.back_name[j++] = src_string[i-1];
        				}
        				if(k == 2 && begin) {//front_name
        					if(src_string[i-1] == '\n')
        						camera_info.front_name[j] = '\0';
        					else
        						camera_info.front_name[j++] = src_string[i-1];
        				}  
        				if(src_string[i-1] == '='){
        					j = 0;
        					begin = 1;
        				}
								if(src_string[i-1] == '\n'){
        					k += 1;
        					j = 0;
        					begin = 0;
        				}
        }
        printk("type=%s\n", camera_info.type);
        printk("back_name=%s\n", camera_info.back_name);
        printk("front_name=%s\n", camera_info.front_name);

        return ret;

}
static int write_device_info(void)
{
        mm_segment_t old_fs;
        int ret = 0;
        struct file *filp = NULL;
        char *camera_type;
        
				if(get_camera_type(&camera_type) == 0){
					detect_print("camera_type=%s\n",camera_type);
				}else{
					strcpy(camera_type, YUV_30W_SENSOR"_"YUV_2M_SENSOR"_double");
					printk("Camera detect err, default_type :%s\n", camera_type);
				}
				memset(camera_info.type, 0, sizeof(camera_info.type));
				memset(camera_info.back_name, 0, sizeof(camera_info.back_name));
				memset(camera_info.front_name, 0, sizeof(camera_info.front_name));
				sprintf(camera_info.type, "type=%s\n",camera_type);
				sprintf(camera_info.back_name, "back_name=%s\n",camera_detector->camera[0].name);
    		sprintf(camera_info.front_name, "front_name=%s\n",camera_detector->camera[1].name);
        filp = filp_open(FILE_DIR, O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if(!filp || IS_ERR(filp)){
                printk("%s:open error ....IS(filp):%ld\n", __func__,IS_ERR(filp));
                return -1;
        } 
        
        old_fs = get_fs();
        set_fs(get_ds());
        filp->f_op->llseek(filp, 0, 0);
        ret = filp->f_op->write(filp, camera_info.type, strlen(camera_info.type), &filp->f_pos);
        ret = filp->f_op->write(filp, camera_info.back_name, strlen(camera_info.back_name), &filp->f_pos);
        ret = filp->f_op->write(filp, camera_info.front_name, strlen(camera_info.front_name), &filp->f_pos);    
        set_fs(old_fs);
        filp_close(filp, NULL);
        return ret;
}
//gongpiqiang--- for camera info store

//set one gpio 20131118 wzh
int  camera_gpio_set_range(struct gpio_config  *gpio, __u32 i)
{
#if 1
	if(i > 1)
	{
		printk("group set not support yet\n");
		return -ECFGPIN; 
	}

	char pin_name[64];
	unsigned long config;
	if(IS_AXP_PIN(gpio->gpio)){
	/*axp pin config pin attributes individually*/	
		sunxi_gpio_to_name(gpio->gpio, pin_name);
		config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC, gpio->mul_sel);
		pin_config_set(AXP_PINCTRL, pin_name, config);
		if(gpio->data != GPIO_DATA_DEFAULT){
			config = SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT, gpio->data);
			pin_config_set(AXP_PINCTRL, pin_name, config);
		}
	}else{
		/*config pin attributes individually*/
			sunxi_gpio_to_name(gpio->gpio, pin_name);
			config =  SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_FUNC,  gpio->mul_sel);
			pin_config_set(SUNXI_PINCTRL, pin_name, config);

			if(gpio->pull != GPIO_PULL_DEFAULT)
			{
				config =  SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_PUD,  gpio->pull);
				pin_config_set(SUNXI_PINCTRL, pin_name, config);
			}
			if(gpio->drv_level != GPIO_DRVLVL_DEFAULT)
			{
				config =  SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DRV,  gpio->drv_level);
				pin_config_set(SUNXI_PINCTRL, pin_name, config);
			}
			if(gpio->data!= GPIO_DATA_DEFAULT)
			{
				config =  SUNXI_PINCFG_PACK(SUNXI_PINCFG_TYPE_DAT,  gpio->data);
				pin_config_set(SUNXI_PINCTRL, pin_name, config);
			}
	}
	return 0;
#endif
}


static __s32 camera_req_mclk_pin(__u32 csi_index)
{
    char *csi_para[2] = {"csi0", "csi1"};
    int	                        req_status;
	script_item_u                   item;
	script_item_value_type_e        type;   


        /* 获取gpio list */
	type = script_get_item(csi_para[csi_index], "vip_csi_mck", &item);
	if(SCIRPT_ITEM_VALUE_TYPE_PIO != type) {
		camera_err("script_get_item return type err\n");
		return -ECFGERR;
	}

        /* 申请gpio */
		
	req_status = gpio_request(item.gpio.gpio, NULL);
	if(0 != req_status) {
		camera_err("request gpio failed\n");
        return -ENORMALPIN;
	}

		/* 配置gpio */
	if(0 != camera_gpio_set_range(&item.gpio, 1)) {
		camera_err("sw_gpio_setall_range failed\n");
        gpio_free(item.gpio.gpio);
        return -ECFGPIN;
	}

        /* 释放gpio */
	if(0 == req_status)
		gpio_free(item.gpio.gpio);
        return 0;


}

static __s32 camera_request_clk(__u32 csi_index,
                                        struct clk **csi_module_clk, 
                                        struct clk **csi_clk_src, 
                                        __hdle *csi_pin_hd)
{
	//only one csi
					
    char *csi[2] = {VFE_MASTER_CLK0, VFE_MASTER_CLK1};
    __s32 ret = 0;

    //*csi_pin_hd = gpio_request_ex(csi_para[csi_index], NULL);
    
    ret = camera_req_mclk_pin(csi_index);
    if(ret != 0) {
        camera_err("request Mclock fail !!\n");
        return ret;
    }
    *csi_module_clk = clk_get(NULL, csi[csi_index]);
    if(*csi_module_clk == NULL) {
    	camera_err("get %s module clk error!\n", csi[csi_index]);
    	return -ECLKSRC;
    }

    *csi_clk_src = clk_get(NULL, VFE_MASTER_CLK_24M_SRC);//"hosc"
    if (*csi_clk_src == NULL) {
        camera_err("get %s hosc source clk error!\n", csi[csi_index]);	
    	return -ECLKSRC;
    }

    ret = clk_set_parent(*csi_module_clk, *csi_clk_src);
    if (ret == -1) {
        camera_err(" csi set parent failed \n");
        return -ECLKSRC;
    }
    clk_put(*csi_clk_src);
    ret = clk_set_rate(*csi_module_clk, CSI_MCLK);
    if (ret == -1) {
    	camera_err("set %s module clock error\n", csi[csi_index]);
    	return -ECLKSRC;
    }

	ret = clk_prepare_enable(*csi_module_clk);
    if (ret == -1) {
        camera_err("enable module clock error\n");
        return -ECLKSRC;
    }
    return 0;
}

static void camera_release_clk(struct clk **csi_module_clk)
{
    clk_disable(csi_module_clk);
    clk_put(csi_module_clk);
    *csi_module_clk = NULL;
}

static __s32 camera_mclk_open(__camera_detector_t *camera_detector)
{
//    __u32 i, csi_cnt = 0;

    detect_print("camera_mclk_open !!\n");

     if (camera_sub_name_exist("csi0", "vip_used")) 
	{ 
		camera_request_clk(0,    &camera_detector->camera[0].module_clk, 
			&camera_detector->camera[0].clk_src,
			&camera_detector->camera[0].clk_pin_hdle); 
	}

    if (camera_sub_name_exist("csi1", "vip_used")) {
        camera_request_clk(1, &camera_detector->camera[1].module_clk, 
                         &camera_detector->camera[1].clk_src, 
                         &camera_detector->camera[1].clk_pin_hdle);
    }

    return 0;
}

void camera_mclk_close(__camera_detector_t *camera_detector)
{
    detect_print("camera_mclk_close !!\n");

    if (camera_detector->camera[0].module_clk != NULL) {
        camera_release_clk(camera_detector->camera[0].module_clk);
    }

    if (camera_detector->camera[1].module_clk != NULL) {
        camera_release_clk(camera_detector->camera[1].module_clk);
    }
}

__bool camera_sub_name_exist(char *main_name, char *sub_name)
{
    script_item_u value;
    script_item_value_type_e ret;
    
    ret = script_get_item(main_name, sub_name, &value);
    if ((SCIRPT_ITEM_VALUE_TYPE_INT == ret) && (value.val == 1)) {
        return TRUE;
    }
    else {
        return FALSE;
    }
}

__s32 camera_get_sysconfig(char *main_name, char *sub_name, __s32 *buf, __u32 cnt)
{
    script_item_u   val;
    script_item_value_type_e  type;
    int ret = -1;

    if(cnt == 1)
    {
		type = script_get_item(main_name, sub_name, &val);
        if(SCIRPT_ITEM_VALUE_TYPE_INT == type)
        {
			ret = 0;
            *buf = val.val;
            detect_print("%s.%s=%d\n", main_name, sub_name, *buf);
        }
        else
        {
            ret = -1;
            detect_print("fetch script data %s.%s fail\n", main_name, sub_name);
        }
    }
    else if(cnt == sizeof(__camera_gpio_set_t)/sizeof(__s32))
    {
		type = script_get_item(main_name, sub_name, &val);
        if(SCIRPT_ITEM_VALUE_TYPE_PIO == type)
        {
			__camera_gpio_set_t *gpio_info = (__camera_gpio_set_t *)buf;

            ret = 0;
            gpio_info->gpio = val.gpio.gpio;
            gpio_info->mul_sel = val.gpio.mul_sel;
            gpio_info->pull = val.gpio.pull;
            gpio_info->drv_level = val.gpio.drv_level;
            gpio_info->data = val.gpio.data;
            memcpy(gpio_info->gpio_name, sub_name, strlen(sub_name)+1);
            detect_print("%s.%s gpio=%d,mul_sel=%d,data:%d\n",main_name, sub_name, gpio_info->gpio, gpio_info->mul_sel, gpio_info->data);
        }
        else
        {
            ret = -1;
            detect_print("fetch script data %s.%s fail\n", main_name, sub_name);
        }
    }
    else if(cnt == MAX_VDD_STR_LEN)
    {
        type = script_get_item(main_name, sub_name, &val);
        if(SCIRPT_ITEM_VALUE_TYPE_STR == type)
        {
            strcpy((char *)buf, val.str);
            detect_print("%s.%s=%s\n", main_name, sub_name, (char *)buf);
        }
        else
        {
            ret = -1;
            detect_print("fetch script data %s.%s fail\n", main_name, sub_name);
        }
    }

    return ret;
}

/*
use_b_para:
TRUE:  use para from [csi0_para] _b para
FALSE: NOT use para from [csi0_para] _b para
*/
static __s32 camera_get_para(__camera_detector_t *camera_detector,
                                    __u32 camera_index,
                                    char *main_name,
                                    __bool use_b_para)
{
    __u32 para_index;
    __u32 pin_struct_size;
    //char csi_drv_node[2][MAX_NODE_NAME_LEN] = {"/dev/video0", "/dev/video1"};  
    char csi_drv_node[2][MAX_NODE_NAME_LEN] = {"sun8i_vip0", "sun8i_vip1"};  
//    __s32 ret;

    pin_struct_size = sizeof(__camera_gpio_set_t)/sizeof(__s32);

    if (strcmp(main_name, "csi0") == 0) {
        para_index = 0;
    }
    else {
        para_index = 1;
    }

    memcpy(camera_detector->camera[camera_index].drv_node_name, csi_drv_node[para_index], MAX_NODE_NAME_LEN);

    if (use_b_para) {
        camera_get_sysconfig(main_name, "vip_dev1_twi_id",
                        &(camera_detector->camera[camera_index].i2c_id), 1);
	#if 0
        ret = camera_get_sysconfig(main_name, "csi_facing_b",
                        (__s32 *)(&(camera_detector->camera[camera_index].facing)), 1);
        if (ret < 0) {
            detect_print("if you want to use camera detector, "
                    "please add csi_facing_b in the [csi0_para]!! \n");
        }
	#endif
		camera_get_sysconfig(main_name, "vip_dev1_reset",
	                            (__s32 *)camera_detector->camera[camera_index].reset_pin, 
	                            pin_struct_size);
	    camera_get_sysconfig(main_name, "vip_dev1_power_en",
	                            (__s32 *)camera_detector->camera[camera_index].pwr_pin, 
	                            pin_struct_size);
	    camera_get_sysconfig(main_name, "vip_dev1_pwdn",
	                            (__s32 *)camera_detector->camera[camera_index].stby_pin, 
	                            pin_struct_size);
	    camera_get_sysconfig(main_name, "vip_dev1_af_pwdn",
	                            (__s32 *)camera_detector->camera[camera_index].af_pwr_pin, 
	                            pin_struct_size);
	    camera_get_sysconfig(main_name, "vip_dev1_iovdd",
	                            (__s32 *)camera_detector->camera[camera_index].iovdd_str, 
	                            MAX_VDD_STR_LEN);
		camera_get_sysconfig(main_name, "vip_dev1_iovdd_vol",
	                            &(camera_detector->camera[camera_index].iovdd_vol), 
	                            1);
	    camera_get_sysconfig(main_name, "vip_dev1_avdd",
	                            (__s32 *)camera_detector->camera[camera_index].avdd_str, 
	                            MAX_VDD_STR_LEN);
		camera_get_sysconfig(main_name, "vip_dev1_avdd_vol",
	                            &(camera_detector->camera[camera_index].avdd_vol), 
	                            1);
		camera_get_sysconfig(main_name, "vip_dev1_dvdd",
	                            (__s32 *)camera_detector->camera[camera_index].dvdd_str, 
	                            MAX_VDD_STR_LEN);

		
		camera_get_sysconfig(main_name, "vip_dev1_dvdd_vol",
	                            &(camera_detector->camera[camera_index].dvdd_vol), 
	                            1);
		//wzh 20131128 do not change dvdd_str
		/*
	    camera_get_sysconfig(main_name, "vip_dev1_afvdd",
	                            (__s32 *)camera_detector->camera[camera_index].dvdd_str, 
	                            MAX_VDD_STR_LEN);
		*/
    }
    else {
        camera_get_sysconfig(main_name, "vip_dev0_twi_id",
                                &(camera_detector->camera[camera_index].i2c_id), 1);
        camera_get_sysconfig(main_name, "vip_dev0_reset",
                                (__s32 *)camera_detector->camera[camera_index].reset_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "vip_dev0_power_en",
                                (__s32 *)camera_detector->camera[camera_index].pwr_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "vip_dev0_pwdn",
                                (__s32 *)camera_detector->camera[camera_index].stby_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "vip_dev0_af_pwdn",
                                (__s32 *)camera_detector->camera[camera_index].af_pwr_pin, 
                                pin_struct_size);
        camera_get_sysconfig(main_name, "vip_dev0_iovdd",
                                (__s32 *)camera_detector->camera[camera_index].iovdd_str, 
                                MAX_VDD_STR_LEN);
		camera_get_sysconfig(main_name, "vip_dev0_iovdd_vol",
                                &(camera_detector->camera[camera_index].iovdd_vol), 
                                1);
        camera_get_sysconfig(main_name, "vip_dev0_avdd",
                                (__s32 *)camera_detector->camera[camera_index].avdd_str, 
                                MAX_VDD_STR_LEN);
		camera_get_sysconfig(main_name, "vip_dev0_avdd_vol",
                                &(camera_detector->camera[camera_index].avdd_vol), 
                                1);
   		camera_get_sysconfig(main_name, "vip_dev0_dvdd",
                        		(__s32 *)camera_detector->camera[camera_index].dvdd_str, 
                        		MAX_VDD_STR_LEN);
		
		camera_get_sysconfig(main_name, "vip_dev0_dvdd_vol",
                                &(camera_detector->camera[camera_index].dvdd_vol), 
                                1);
		//		//wzh 20131128 do not change dvdd_str
		/*
        camera_get_sysconfig(main_name, "vip_dev0_afvdd",
                                (__s32 *)camera_detector->camera[camera_index].dvdd_str, 
                                MAX_VDD_STR_LEN);
		*/
    }




    if (camera_detector->camera[camera_index].reset_pin->port != 0) {
        camera_detector->camera[camera_index].reset_pin_used = 1;
    }
    if (camera_detector->camera[camera_index].pwr_pin->port != 0) {
        camera_detector->camera[camera_index].pwr_pin_used = 1;
    }
    if (camera_detector->camera[camera_index].stby_pin->port != 0) {
        camera_detector->camera[camera_index].stby_pin_used = 1;
    }

    if (strcmp(camera_detector->camera[camera_index].iovdd_str, "")) {
        camera_detector->camera[camera_index].iovdd = 
                        regulator_get(NULL, camera_detector->camera[camera_index].iovdd_str);
	    if (IS_ERR(camera_detector->camera[camera_index].iovdd)) {
	        camera_err("get regulator csi_iovdd error!! \n");
	        return -EPMUPIN;
	    }

    }
    if (strcmp(camera_detector->camera[camera_index].avdd_str, "")) {
        camera_detector->camera[camera_index].avdd = 
                        regulator_get(NULL, camera_detector->camera[camera_index].avdd_str);
        if (IS_ERR(camera_detector->camera[camera_index].avdd)) {
            camera_err("get regulator csi_avdd error!! \n");
            return -EPMUPIN;
        }
    }
    if (strcmp(camera_detector->camera[camera_index].dvdd_str, "")) {
        camera_detector->camera[camera_index].dvdd = 
                        regulator_get(NULL, camera_detector->camera[camera_index].dvdd_str);
        if (IS_ERR(camera_detector->camera[camera_index].dvdd)) {
                camera_err("get regulator csi_dvdd error!! \n");
                return -EPMUPIN;
        }
    }

	return 0;
}

static __s32 camera_get_board_info(__camera_detector_t *camera_detector)
{
    char *csi[2] = {"csi0", "csi1"};
    __s32 value;
    __s32 ret;

    //get camera number
    if (camera_sub_name_exist(csi[0], "vip_used") 
            && camera_sub_name_exist(csi[1], "vip_used")) {
				camera_detector->num = 2;
				camera_get_para(camera_detector, 0, csi[0], FALSE);
				camera_get_para(camera_detector, 1, csi[1], FALSE);
    }
	
    else if (camera_sub_name_exist(csi[0], "vip_used")) {
        ret = camera_get_sysconfig(csi[0], "vip_dev_qty", &value, 1);
        if (ret < 0) {
            return -ECFGNOEX;
        }
        else 
				{
						camera_detector->num = value;
						//wzh test
						camera_detector->num = 2;
						if(camera_detector->num == 2)
						{
							camera_get_para(camera_detector, 0, csi[0], FALSE);
							camera_get_para(camera_detector, 1, csi[0], TRUE);
						}
						else if(camera_detector->num == 1) {
							camera_get_para(camera_detector, 0, csi[0], FALSE);
						}
						else {
					        return -ECAMNUMERR;
				            }
        }
    }
	else if (camera_sub_name_exist(csi[1], "vip_used")) {
        ret = camera_get_sysconfig(csi[1], "vip_dev_qty", &value, 1);
        if (ret < 0) {
            return -ECFGNOEX;
        }
        else {
			camera_detector->num = value;
			if(camera_detector->num == 2)
			{
				camera_get_para(camera_detector, 0, csi[1], FALSE);
				camera_get_para(camera_detector, 1, csi[1], TRUE);
			}
			else if(camera_detector->num == 1) {
				camera_get_para(camera_detector, 0, csi[1], FALSE);
			}
			else {
		        return -ECAMNUMERR;
            }
        }
    }
    else {
		return -ECFGERR;
    }

    camera_detector->camera[0].i2c_adap = i2c_get_adapter(camera_detector->camera[0].i2c_id);
    if (camera_detector->camera[0].i2c_adap == NULL) {
        camera_err("get I2C adapter fail, I2C id: %d \n", camera_detector->camera[0].i2c_id);
		return -EI2CADAPTER;
    }
    if (camera_detector->num == 2) {
        if (camera_detector->camera[0].i2c_id != camera_detector->camera[1].i2c_id) {
            camera_detector->camera[1].i2c_adap = i2c_get_adapter(camera_detector->camera[1].i2c_id);
            if (camera_detector->camera[1].i2c_adap == NULL) {
                camera_err("get I2C adapter fail, I2C id: %d \n", camera_detector->camera[1].i2c_id);
				return -EI2CADAPTER;
            } 
        }
        else {
            camera_detector->camera[1].i2c_adap = camera_detector->camera[0].i2c_adap;
        }
    }

return 0;
}

static __s32 camera_init_module_list(__u32 camera_list_size)
{
    __u32 i;

    detect_print("camera_list_size: %d \n", camera_list_size);

    for (i = 0; i < camera_list_size; i++) {
        if (camera_sub_name_exist("camera_list_para", camera_list[i].name)) {
            camera_list[i].need_detect = TRUE;
            printk("modules: %s need detect!!\n", camera_list[i].name);
        }
    }

    return 0;
}

static __s32 camera_diff_i2c_id_detect(__camera_detector_t *camera_detector, 
                                                __camera_list_t *camera_list,
                                                __u32 camera_list_size)
{
    __u32 i, j;
    __u32 camera_detected = 0;
    __s32 ret = 0;

    detect_print("camera_diff_i2c_id_detect!!\n");
    
    for (i = 0; i < camera_detector->num; i++) {
        for (j = 0; j < camera_list_size; j++) {
            if (camera_list[j].need_detect) {
                camera_list[j].pwr_on(j, &camera_detector->camera[i]);
                ret = camera_list[j].detect(j, camera_detector->camera[i].i2c_adap);
                if (ret == 0) {
                	//camera_list[j].pwr_off(j, &camera_detector->camera[i]);
                    strcpy(camera_detector->camera[i].name, camera_list[j].name);
                    camera_detector->camera[i].i2c_addr = camera_list[j].i2c_addr;
                    camera_detected++;
                    break;
                }
            }
        }
    }

	for (i = 0; i < camera_detector->num; i++) {
		unsigned int a,b,c;
		a=b=c=0;
		if(camera_detector->camera[i].iovdd) {
			while(regulator_is_enabled(camera_detector->camera[i].iovdd))
			{
				a++;
				regulator_disable(camera_detector->camera[i].iovdd);
				//printk("cam[%d]iovdd count=%d\n",i,a);
				if(a>100)
					break;
			}
			regulator_put(camera_detector->camera[i].iovdd);
		}
		if(camera_detector->camera[i].avdd) {
			while(regulator_is_enabled(camera_detector->camera[i].avdd))
			{
				b++;
				regulator_disable(camera_detector->camera[i].avdd);
				//printk("cam[%d]avdd count=%d\n",i,b);
				if(b>100)
					break;
			}
			regulator_put(camera_detector->camera[i].avdd);
		}
		if(camera_detector->camera[i].dvdd) {
			while(regulator_is_enabled(camera_detector->camera[i].dvdd))
			{
				c++;
				regulator_disable(camera_detector->camera[i].dvdd);
				//printk("cam[%d]dvdd count=%d\n",i,c);
				if(c>100)
					break;
			}
			regulator_put(camera_detector->camera[i].dvdd);
		}
	}
	
    if (camera_detector->num != camera_detected) {
        camera_err("detect camera fail in func: camera_diff_i2c_id_detect !!\n");
        return -EDETECTFAIL;
    }
	
    return 0;
}

void change_pwdn(__camera_detector_t *camera_detector)
{
	char name_temp[64];
	__u32 i2c_addr_temp;
	
	strcpy(name_temp, camera_detector->camera[0].name);
	i2c_addr_temp = camera_detector->camera[0].i2c_addr;
		
		printk("name_temp[%s] i2c_addr_temp[%#x]\n", name_temp, i2c_addr_temp);
	
	
	
	strcpy(camera_detector->camera[0].name,camera_detector->camera[1].name);
	camera_detector->camera[0].i2c_addr = camera_detector->camera[1].i2c_addr;
	
	printk("camera[0] name_temp[%s] icamera[0] 2c_addr_temp[%#x]\n", camera_detector->camera[0].name, camera_detector->camera[0].i2c_addr);
	
	strcpy(camera_detector->camera[1].name, name_temp);
	camera_detector->camera[1].i2c_addr = i2c_addr_temp;
	
		printk("camera[1] name_temp[%s] icamera[1] 2c_addr_temp[%#x]\n", camera_detector->camera[1].name, camera_detector->camera[1].i2c_addr);
}


static __s32 camera_same_i2c_id_detect(__camera_detector_t *camera_detector, 
                                                  __camera_list_t *camera_list,
                                                  __u32 camera_list_size)
{	
    __u32 i, j;
	__u32 camera_detected = 0;
    __s32 ret = 0;

    detect_print("camera_same_i2c_id_detect!!\n");

    for (i = 0; i < camera_detector->num; i++) {
        for (j = 0; j < camera_list_size; j++) {
        		if(camera_info_done) {
							if(i==0){//back
								if(!strcmp(camera_info.back_name, camera_list[j].name)){
									printk("back camera setted,back_name=%s\n",camera_list[j].name);
									camera_detected++;
									single_facing = i;//gongpiqiang+++ 如果是单摄像头，是前置还是后置标志	
									strcpy(camera_detector->camera[i].name, camera_list[j].name);
								  camera_detector->camera[i].i2c_addr = camera_list[j].i2c_addr;
								  break;
								}
							}else{
								if(!strcmp(camera_info.front_name, camera_list[j].name)){
									printk("front camera setted,front_name=%s\n",camera_list[j].name);
									camera_detected++;
									single_facing = i;//gongpiqiang+++ 如果是单摄像头，是前置还是后置标志	
									strcpy(camera_detector->camera[i].name, camera_list[j].name);
								  camera_detector->camera[i].i2c_addr = camera_list[j].i2c_addr;
								  break;
								}
							}
							
        	 }
		       else if (camera_list[j].need_detect) {
						    camera_stby_on_sensor(j,&camera_detector->camera[0]);
						    camera_stby_on_sensor(j,&camera_detector->camera[1]);
			          camera_list[j].pwr_on(j, &camera_detector->camera[i]);

			          ret = camera_list[j].detect(j, camera_detector->camera[i].i2c_adap);
			          if (ret == 0) {
										//camera_list[j].need_detect=false;
										camera_detected++;
										single_facing = i;//gongpiqiang+++ 如果是单摄像头，是前置还是后置标志
					                	//camera_list[j].pwr_off(j, &camera_detector->camera[i]);
										strcpy(camera_detector->camera[i].name, camera_list[j].name);
										camera_detector->camera[i].i2c_addr = camera_list[j].i2c_addr;
										break;
								}
				    }
        }
    }
		if(camera_detected == 2 && strcmp(camera_detector->camera[0].name, camera_detector->camera[1].name)
  	                      && camera_detector->camera[0].i2c_addr == camera_detector->camera[1].i2c_addr)
		{
				printk("diff sensor, same i2c addr. need fake i2c address");
				camera_detector->camera[1].i2c_addr = 0x88;
		}
	
	for (i = 0; i < camera_detector->num; i++) {
		unsigned int a,b,c;
		a=b=c=0;
		if(camera_detector->camera[i].iovdd) {
			while(regulator_is_enabled(camera_detector->camera[i].iovdd))
			{
				a++;
				regulator_disable(camera_detector->camera[i].iovdd);
				//printk("cam[%d]iovdd count=%d\n",i,a);
				if(a>100)
					break;
			}
			regulator_put(camera_detector->camera[i].iovdd);
		}
		if(camera_detector->camera[i].avdd) {
			while(regulator_is_enabled(camera_detector->camera[i].avdd))
			{
				b++;
				regulator_disable(camera_detector->camera[i].avdd);
				//printk("cam[%d]avdd count=%d\n",i,b);
				if(b>100)
					break;
			}
			regulator_put(camera_detector->camera[i].avdd);
		}
		if(camera_detector->camera[i].dvdd) {
			while(regulator_is_enabled(camera_detector->camera[i].dvdd))
			{
				c++;
				regulator_disable(camera_detector->camera[i].dvdd);
				//printk("cam[%d]dvdd count=%d\n",i,c);
				if(c>100)
					break;
			}
			regulator_put(camera_detector->camera[i].dvdd);
		}
	}
    
    if (camera_detector->num != camera_detected)  {
        //camera_err("detect camera fail in func: camera_same_i2c_id_detect !!\n");
        //return -EDETECTFAIL;
        //gongpiqiang+++
        printk("expect camera_num=%d,but camera_detected=%d\n",camera_detector->num,camera_detected);
        camera_detector->num = camera_detected;
        if(camera_detector->num == 0)
        	return -EDETECTFAIL;
        //gongpiqiang---
    }
	
	detect_print("camera_detector->camera[0].name=%s,camera_detector->camera[1].name=%s\n",camera_detector->camera[0].name,camera_detector->camera[1].name);
	
	//d78 double camera, need exchange standby_pin wzh 20140519
		script_item_u   val;
 		script_item_value_type_e	type;
 		int pwdn_change;
		type = script_get_item("csi0","pwdn_change", &val);
		if (SCIRPT_ITEM_VALUE_TYPE_INT != type) {
			pwdn_change = 0;
		  printk("fetch pwdn_change from sys_config failed\n");
		} else {
		  pwdn_change=val.val;
		  printk("pwdn_change===============%d\n",pwdn_change);
		}
		if(pwdn_change == 1 && camera_detector->num == 2)
			change_pwdn(camera_detector);

	return 0;
}

static __u32 camera_get_facing_index(__camera_detector_t *camera_detector, 
                                               __camera_facing_e camera_facing)
{
    if (camera_detector->num == 1) {
        if (camera_facing == CAMERA_FACING_BACK) {
            return 0;
        }
        if (camera_facing == CAMERA_FACING_FRONT) {
            return 1;
        }
    }
    else {
        if (camera_detector->camera[0].facing == camera_facing) {
            return 0;
        }
        if (camera_detector->camera[1].facing == camera_facing) {
            return 1;
        }
    }

	return 0;
}

static ssize_t camera_num_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    __u32 camera_num;

    if ((camera_detector->num == 1) && 
		( strcmp(camera_detector->camera[0].name, "") || strcmp(camera_detector->camera[1].name, ""))
		) 
	{
        camera_num = 1;
    }
    else if ((camera_detector->num == 2) 
        && strcmp(camera_detector->camera[0].name, "")
        && strcmp(camera_detector->camera[1].name, "")) {
        camera_num = 2;
    }
    else {
        camera_num = 0;
    }

    return sprintf(buf, "%u", camera_num);
}

static ssize_t camera_num_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
    return 0;
}

static ssize_t camera_front_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	__u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_FRONT);
	if(camera_detector->num == 2)//gongpiqiang+++   
		index =1 ;
	return sprintf(buf, camera_detector->camera[index].name);
}

static ssize_t camera_front_name_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}

static ssize_t camera_back_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{   
    __u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_BACK);
    return sprintf(buf, camera_detector->camera[index].name);
}

static ssize_t camera_back_name_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}

static ssize_t camera_front_node_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{   
    __u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_FRONT);

    if (strcmp(camera_detector->camera[index].name, "")) {
		return sprintf(buf, camera_detector->camera[index].drv_node_name);
    }
    else {
        *buf = '\0';
        return 0;
    }
}

static ssize_t camera_front_node_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}

static ssize_t camera_back_node_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{   
    __u32 index = camera_get_facing_index(camera_detector, CAMERA_FACING_BACK);

    if (strcmp(camera_detector->camera[index].name, "")) {
		return sprintf(buf, camera_detector->camera[index].drv_node_name);
    }
    else {
        *buf = '\0';
        return 0;
    }
}

static ssize_t camera_back_node_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}
/* 
* 同类型sensor使用相同的"type_name" 20130801 wzh
* 30W->gc0308 |2M->gt2005 |5M->OV5640 |2M_RAW->gc2235 |5M_RAW-> s5k4e1 
*/
int sensor_name_2_camera_type( char *detected_sensor_name,__camera_sensor_info sensor_info)
{
printk("detected_sensor_name=%s,sensor_info.sensor_name=%s,pixels=%d\n",detected_sensor_name,sensor_info.sensor_name,sensor_info.pixels);
	if(strcmp(detected_sensor_name, sensor_info.sensor_name) == 0)
	{
		switch(sensor_info.pixels)
		{			
			case 30:
				strcat(camera_type_buffer, YUV_30W_SENSOR);
				break;
			case 200:
				if(sensor_info.sensor_fmt == YUV)
					strcat(camera_type_buffer, YUV_2M_SENSOR);
				else
					strcat(camera_type_buffer, RAW_2M_SENSOR);
				break;					
			case 500:
				if(sensor_info.sensor_fmt == YUV)
					strcat(camera_type_buffer, YUV_5M_SENSOR);
				else
					strcat(camera_type_buffer, RAW_5M_SENSOR);
				break;
			default:
				printk("unknown pixel\n");
				return -2;
		}
		return 0;
	}
	//printk("unknown sensor type.\n");
	return -1;
}


static int get_camera_type(char **camera_type)
{
	__u32 i, j, sensor_num;
	//char **sensor_name = all_sensor_name;
	__camera_sensor_info *sensor_info = all_sensor_info;
	
	sensor_num= sizeof(all_sensor_info)/sizeof(__camera_sensor_info);	
	memset(camera_type_buffer, 0, sizeof(camera_type_buffer));

	if(camera_detector->num == 2)
	{
		for(i=0; i<(camera_detector->num); i++)
		{
			for(j=0; j<sensor_num; j++)
			{
				// i = 0 -> 1-i == 1 == front
				if(sensor_name_2_camera_type(camera_detector->camera[1-i].name, sensor_info[j]) == 0)
				{
					if(i == 0)	//front
					{
						/* gc0308_ */
						strcat(camera_type_buffer, "_");
					}
					else
					{
						/* gc0308_gt2005_doule */
						strcat(camera_type_buffer, "_double");
					}
					break;
				 }
			 }
			 if(j >= sensor_num)
			 {
			 	return -1;
			 }			
		}
		*camera_type = camera_type_buffer;
		return 0;
	}
	else if(camera_detector->num == 1)
	{
		for(j=0; j<sensor_num; j++)
		{
			//if()
			if(sensor_name_2_camera_type(camera_detector->camera[single_facing].name, sensor_info[j]) == 0)
			{
				if(single_facing == 1)
					strcat(camera_type_buffer, "_front");
				else
					strcat(camera_type_buffer, "_back");
				break;
			}					
		}
		if(j >= sensor_num)
		{
		 	return -1;
		}
		*camera_type = camera_type_buffer;
		return 0;
	}
	else
	{
		return -1;
	}	
}
//gongpiqiang+++ add camera
static ssize_t camera_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{   
		char *camera_type;

		if(get_camera_type(&camera_type) == 0)
		{
			detect_print("camera_type=%s\n",camera_type);
		}
    	 else
		{
			strcpy(camera_type, YUV_30W_SENSOR"_"YUV_2M_SENSOR"_double");
			printk("Camera detect err, default_type :%s\n", camera_type);
		}
    	
    	//return sprintf(buf, "%u", camera_type_index);
    printk("camera_type=%s\n",camera_type);
    return sprintf(buf, camera_type);	
}

static ssize_t camera_type_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}

//isp maybe no used in A23
#ifdef A23_ISP_USED
static ssize_t camera_isp_used_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	return sprintf(buf, "%d",raw_sensor);
}


static ssize_t camera_isp_used_store(struct device *dev,struct device_attribute *attr,
		const char *buf, size_t size)
{
	return 0;
}
#endif


//gongpiqiang--

static DEVICE_ATTR(num, S_IRUGO|S_IWUSR|S_IWGRP, camera_num_show, camera_num_store);
static DEVICE_ATTR(front_name, S_IRUGO|S_IWUSR|S_IWGRP, camera_front_name_show, camera_front_name_store);
static DEVICE_ATTR(back_name, S_IRUGO|S_IWUSR|S_IWGRP, camera_back_name_show, camera_back_name_store);
static DEVICE_ATTR(front_node, S_IRUGO|S_IWUSR|S_IWGRP, camera_front_node_show, camera_front_node_store);
static DEVICE_ATTR(back_node, S_IRUGO|S_IWUSR|S_IWGRP, camera_back_node_show, camera_back_node_store);
static DEVICE_ATTR(type, S_IRUGO|S_IWUSR|S_IWGRP, camera_type_show, camera_type_store);//gongpiqiang
#ifdef A23_ISP_USED
static DEVICE_ATTR(isp_used, S_IRUGO|S_IWUSR|S_IWGRP, camera_isp_used_show, camera_isp_used_store);
#endif

static struct attribute *camera_detector_attributes[] = {
        &dev_attr_num.attr,
        &dev_attr_front_name.attr,
        &dev_attr_back_name.attr,
        &dev_attr_front_node.attr,
        &dev_attr_back_node.attr,
        &dev_attr_type.attr,//gongpiqiang+++
        #ifdef A23_ISP_USED
        &dev_attr_isp_used.attr,  //isp maybe no used in A23
       #endif
        NULL
};

static struct attribute_group dev_attr_group = {
	.attrs = camera_detector_attributes,
};

static const struct attribute_group *dev_attr_groups[] = {
	&dev_attr_group,
	NULL,
};

static void camera_detector_release(struct device *dev)
{
    
}

static struct device camera_detector_device = {
        .init_name = "camera",
        .release = camera_detector_release,
};

static int __init camera_detector_init(void) {
	int err = 0;
    __u32 camera_list_size;
    char *camera_list_para      = "camera_list_para";
    char *camera_list_para_used = "camera_list_para_used";
    
	detect_print("camera detect driver init\n");
    
    if (!camera_sub_name_exist(camera_list_para, camera_list_para_used)) {
            __s32 value, ret;

        ret = camera_get_sysconfig(camera_list_para, camera_list_para_used, &value, 1);
        if (ret < 0) {
			detect_print("[camera_list_para] not exist in sys_config1.fex !! \n");
        }
        else if (value == 0) {
            detect_print("[camera_list_para]->camera_list_para_used = 0," 
                    "maybe somebody does not want to use camera detector ...... \n");
        }

        detect_print("camera detector exit, it will do nothing ...... \n");

        goto exit;
    }
        
	camera_detector_device.groups = dev_attr_groups;
	err = device_register(&camera_detector_device);
	if (err) {
		camera_err("%s register camera detect driver as misc device error\n", __FUNCTION__);
		goto exit;
	}
    
    memset(camera_detector, 0, sizeof(__camera_detector_t));
    err = camera_get_board_info(camera_detector);
    if (err)
    {
        camera_err("camera_get_board_info fail !!\n");
        goto exit;
    }
	
    camera_list_size = sizeof(camera_list) / sizeof(__camera_list_t);
    //camera_init_module_list(camera_list_size);
		camera_info_done = get_device_info();
    camera_mclk_open(camera_detector);

    if ((camera_detector->camera[0].i2c_id == camera_detector->camera[1].i2c_id)
    && (camera_detector->num == 2)) {
		camera_same_i2c_id_detect(camera_detector, camera_list, camera_list_size);
    }
    else if ((camera_detector->camera[0].i2c_id != camera_detector->camera[1].i2c_id)
    || (camera_detector->num == 1)) {
		camera_diff_i2c_id_detect(camera_detector, camera_list, camera_list_size);
    }

    camera_mclk_close(camera_detector);

		if(!camera_info_done)
			write_device_info();
    return 0;
    
exit:
	return err;
}

static void __exit camera_detector_exit(void) {

	detect_print("Bye, camera detect driver exit\n");
	device_unregister(&camera_detector_device);
}

void camera_export_info(char *module_name, int *i2c_addr, int index)
{

  	strcpy(module_name, camera_detector->camera[index].name);
    *i2c_addr = camera_detector->camera[index].i2c_addr; 
}

EXPORT_SYMBOL(camera_export_info);

//isp maybe no used in A23 wzh 20131118+++
#ifdef A23_ISP_USED
static __bool is_raw_sensor(char *sensor_name)
{
	int i;
	int sensor_num= sizeof(all_sensor_info)/sizeof(__camera_sensor_info);	
	
	for(i=0; i<sensor_num; i++)
	{
		if(strcmp(all_sensor_info[i].sensor_name, sensor_name) == 0)
		{
			if(all_sensor_info[i].sensor_fmt > YUV)
				{
					raw_sensor++;
					return TRUE;
				}
			else
				return FALSE;
		}
	}
	return FALSE;
}

__bool __raw_sensor__(int dev_id, int camera_num, char *back_name, char *front_name, int front_back)
{
	if(camera_num == 1)	
	{
		//front_back==1 ->  single front
		return (front_back==1) ? is_raw_sensor(front_name):is_raw_sensor(back_name);
	}

	else if(camera_num == 2)
	{
		//0 for back; 1 for front
		return (dev_id == 1) ? is_raw_sensor(front_name):is_raw_sensor(back_name);		
	}
	else
		return FALSE;
}
EXPORT_SYMBOL(__raw_sensor__);
#endif
//isp maybe no used in A23 wzh 20131118---

//gongpiqiang+++
void camera_export_type_info(int *camera_num, char *back_name, char *front_name, int *front_back)
{
    *camera_num = camera_detector->num;
    *front_back = single_facing;
    strcpy(back_name, camera_detector->camera[0].name);
    strcpy(front_name, camera_detector->camera[1].name);
}

EXPORT_SYMBOL(camera_export_type_info);
//gongpiqiang---

module_init(camera_detector_init);
module_exit(camera_detector_exit);

MODULE_DESCRIPTION("camera detect driver");
MODULE_AUTHOR("heyihang");
MODULE_LICENSE("GPL");

