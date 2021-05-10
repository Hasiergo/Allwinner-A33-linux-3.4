

struct fw_data
{
    u32 offset : 8;
    u32 : 0;
    u32 val;
};

struct gsl_touch_info
{
	int x[10];
	int y[10];
	int id[10];
	int finger_num;	
};

