<<<<<<< HEAD
/* Himax Android Driver Sample Code for Himax chipset
*
* Copyright (C) 2015 Himax Corporation.
=======
/* Himax Android Driver Sample Code for common functions
*
* Copyright (C) 2017 Himax Corporation.
>>>>>>> .
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#ifndef HIMAX_COMMON_H
#define HIMAX_COMMON_H

<<<<<<< HEAD
#include "himax_platform.h"

#include <asm/segment.h>
/*#include <asm/uaccess.h>*/
/*#include <asm/atomic.h>*/
#include <linux/uaccess.h>
#include <linux/atomic.h>
=======
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
>>>>>>> .

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/async.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/buffer_head.h>
#include <linux/wakelock.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
<<<<<<< HEAD
=======
#include "himax_platform.h"
>>>>>>> .

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
<<<<<<< HEAD
#define HIMAX_DRIVER_VER "0.3.1.0"

#define FLASH_DUMP_FILE "/data/user/Flash_Dump.bin"
#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.csv"

=======
#define HIMAX_DRIVER_VER "0.1.92.0_L2300_11"

#define FLASH_DUMP_FILE "/sdcard/HX_Flash_Dump.bin"

//#define CONFIG_TOUCHSCREEN_HIMAX_DEBUG
>>>>>>> .
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)

#define HX_TP_PROC_DIAG
#define HX_TP_PROC_REGISTER
#define HX_TP_PROC_DEBUG
#define HX_TP_PROC_FLASH_DUMP
#define HX_TP_PROC_SELF_TEST
#define HX_TP_PROC_RESET
#define HX_TP_PROC_SENSE_ON_OFF
<<<<<<< HEAD
/*#define HX_TP_PROC_2T2R*/
=======
#define HX_TP_PROC_2T2R

#ifdef HX_TP_PROC_SELF_TEST
//#define HX_TP_SELF_TEST_DRIVER //if enable, selftest works in driver
#endif
>>>>>>> .

int himax_touch_proc_init(void);
void himax_touch_proc_deinit(void);
#endif
<<<<<<< HEAD

/*===========Himax Option function=============*/
/*#define HX_RST_PIN_FUNC*/
#define HX_AUTO_UPDATE_FW
/*#define HX_HIGH_SENSE*/
/*#define HX_SMART_WAKEUP*/
/*#define HX_USB_DETECT*/
/*#define HX_ESD_WORKAROUND*/
/*#define HX_USB_DETECT2*/
/*#define HX_EN_SEL_BUTTON*//* Support Self Virtual key ,default is close*/
#define HX_EN_MUT_BUTTON/*  Support Mutual Virtual Key ,default is close*/
/*#define HX_EN_CHECK_PATCH*/

#define HX_KEY_MAX_COUNT  4
#define DEFAULT_RETRY_CNT 3

#define HX_VKEY_0 KEY_BACK
#define HX_VKEY_1 KEY_HOME
#define HX_VKEY_2 KEY_RESERVED
#define HX_VKEY_3 KEY_RESERVED
#define HX_KEY_ARRAY {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}

#define SHIFTBITS 5
/*#define FLASH_SIZE 131072*/
#define  FW_SIZE_60k  61440
#define  FW_SIZE_64k  65536
#define  FW_SIZE_124k 126976
#define  FW_SIZE_128k 131072

struct himax_ic_data {
	int vendor_fw_ver;
	int vendor_config_ver;
=======
//===========Himax Option function=============
#define HX_RST_PIN_FUNC
#define HX_AUTO_UPDATE_FW
#define HX_ESD_RECOVERY
//#define HX_CHIP_STATUS_MONITOR		/*for ESD 2nd solution,it does not support incell,default off*/
#define HX_SMART_WAKEUP
#define HX_RESEND_CMD
//#define HX_GESTURE_TRACK
//#define HX_HIGH_SENSE
//#define HX_PALM_REPORT
#define HX_USB_DETECT_GLOBAL
//#define HX_USB_DETECT_CALLBACK
//#define HX_LOADIN_CONFIG
//#define HX_PROTOCOL_A					/* for MTK special platform.If turning on,it will report to system by using specific format. */
//#define HX_RESUME_HW_RESET
#define HX_PROTOCOL_B_3PA
//#define HX_FIX_MAX_PT

//#define HX_EN_SEL_BUTTON		       	/* Support Self Virtual key		,default is close*/
//#define HX_EN_MUT_BUTTON		       	/* Support Mutual Virtual Key	,default is close*/

#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
//#define HX_PLATFOME_DEFINE_KEY		/* for specfic platform to set key(button) */
#endif

#define HX_KEY_MAX_COUNT             4			
#define DEFAULT_RETRY_CNT            3

#define HX_85XX_A_SERIES_PWON		1
#define HX_85XX_B_SERIES_PWON		2
#define HX_85XX_C_SERIES_PWON		3
#define HX_85XX_D_SERIES_PWON		4
#define HX_85XX_E_SERIES_PWON		5
#define HX_85XX_ES_SERIES_PWON		6
#define HX_85XX_F_SERIES_PWON		7
#define HX_83100A_SERIES_PWON		8
#define HX_83102A_SERIES_PWON		9
#define HX_83102B_SERIES_PWON		10
#define HX_83110A_SERIES_PWON		11
#define HX_83110B_SERIES_PWON		12
#define HX_83111B_SERIES_PWON		13
#define HX_83112A_SERIES_PWON		14
#define HX_83112B_SERIES_PWON		15


#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC		3

#define HX_VKEY_0   KEY_BACK
#define HX_VKEY_1   KEY_HOME
#define HX_VKEY_2   KEY_RESERVED
#define HX_VKEY_3   KEY_RESERVED
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}

#define SHIFTBITS 5

#define  FW_SIZE_32k 	32768
#define  FW_SIZE_60k 	61440
#define  FW_SIZE_64k 	65536
#define  FW_SIZE_124k 	126976
#define  FW_SIZE_128k 	131072

#define NO_ERR 0
#define READY_TO_SERVE 1
#define WORK_OUT	2
#define I2C_FAIL -1
#define MEM_ALLOC_FAIL -2
#define CHECKSUM_FAIL -3
#define GESTURE_DETECT_FAIL -4
#define INPUT_REGISTER_FAIL -5
#define FW_NOT_READY -6
#define LENGTH_FAIL -7
#define OPEN_FILE_FAIL -8
#define ERR_WORK_OUT	-10

#define HX_FINGER_ON	1
#define HX_FINGER_LEAVE	2

#define HX_REPORT_COORD	1
#define HX_REPORT_SMWP_EVENT	2

struct himax_ic_data {
	int vendor_fw_ver;
	int himax_calculateChecksum;
	int vendor_config_ver;
    int vendor_touch_cfg_ver;
    int vendor_display_cfg_ver;
	int vendor_cid_maj_ver;
	int vendor_cid_min_ver;
	int vendor_panel_ver;
>>>>>>> .
	int vendor_sensor_id;
	int		HX_RX_NUM;
	int		HX_TX_NUM;
	int		HX_BT_NUM;
	int		HX_X_RES;
	int		HX_Y_RES;
	int		HX_MAX_PT;
	bool	HX_XY_REVERSE;
	bool	HX_INT_IS_EDGE;
#ifdef HX_TP_PROC_2T2R
	int HX_RX_NUM_2;
	int HX_TX_NUM_2;
#endif
};

struct himax_virtual_key {
	int index;
	int keycode;
	int x_range_min;
	int x_range_max;
	int y_range_min;
	int y_range_max;
};

<<<<<<< HEAD
struct himax_config {
	uint8_t  default_cfg;
	uint8_t  sensor_id;
	uint8_t  fw_ver;
	uint16_t length;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
	uint8_t c1[11];
	uint8_t c2[11];
	uint8_t c3[11];
	uint8_t c4[11];
	uint8_t c5[11];
	uint8_t c6[11];
	uint8_t c7[11];
	uint8_t c8[11];
	uint8_t c9[11];
	uint8_t c10[11];
	uint8_t c11[11];
	uint8_t c12[11];
	uint8_t c13[11];
	uint8_t c14[11];
	uint8_t c15[11];
	uint8_t c16[11];
	uint8_t c17[11];
	uint8_t c18[17];
	uint8_t c19[15];
	uint8_t c20[5];
	uint8_t c21[11];
	uint8_t c22[4];
	uint8_t c23[3];
	uint8_t c24[3];
	uint8_t c25[4];
	uint8_t c26[2];
	uint8_t c27[2];
	uint8_t c28[2];
	uint8_t c29[2];
	uint8_t c30[2];
	uint8_t c31[2];
	uint8_t c32[2];
	uint8_t c33[2];
	uint8_t c34[2];
	uint8_t c35[3];
	uint8_t c36[5];
	uint8_t c37[5];
	uint8_t c38[9];
	uint8_t c39[14];
	uint8_t c40[159];
	uint8_t c41[99];
};

struct himax_ts_data {
	bool suspended;
	bool probe_done;
	struct mutex fb_mutex;
	struct mutex rw_lock;
=======
struct himax_report_data {
	int touch_all_size;
	int raw_cnt_max;
	int raw_cnt_rmd;
	int touch_info_size;
	uint8_t	finger_num;
	uint8_t	finger_on;
	uint8_t *hx_coord_buf;
	uint8_t hx_state_info[2];
#if defined(HX_SMART_WAKEUP)
	int event_size;
	uint8_t *hx_event_buf;
#endif
#if defined(HX_TP_PROC_DIAG)
	int rawdata_size;
	uint8_t diag_cmd;
	uint8_t *hx_rawdata_buf;
	uint8_t rawdata_frame_size;
#endif
};


struct himax_ts_data {
	bool suspended;
>>>>>>> .
	atomic_t suspend_mode;
	uint8_t x_channel;
	uint8_t y_channel;
	uint8_t useScreenRes;
	uint8_t diag_command;
<<<<<<< HEAD
=======
	
>>>>>>> .
	uint8_t protocol_type;
	uint8_t first_pressed;
	uint8_t coord_data_size;
	uint8_t area_data_size;
<<<<<<< HEAD
=======
	uint8_t coordInfoSize;
>>>>>>> .
	uint8_t raw_data_frame_size;
	uint8_t raw_data_nframes;
	uint8_t nFinger_support;
	uint8_t irq_enabled;
	uint8_t diag_self[50];
<<<<<<< HEAD
	uint16_t finger_pressed;
	uint16_t last_slot;
	uint16_t pre_finger_mask;
=======
	
	uint16_t finger_pressed;
	uint16_t last_slot;
	uint16_t pre_finger_mask;

>>>>>>> .
	uint32_t debug_log_level;
	uint32_t widthFactor;
	uint32_t heightFactor;
	uint32_t tw_x_min;
	uint32_t tw_x_max;
	uint32_t tw_y_min;
	uint32_t tw_y_max;
	uint32_t pl_x_min;
	uint32_t pl_x_max;
	uint32_t pl_y_min;
	uint32_t pl_y_max;
<<<<<<< HEAD

	int use_irq;
	int (*power)(int on);
	int pre_finger_data[10][2];

=======
	
	int rst_gpio;
	int use_irq;
	int (*power)(int on);
	int pre_finger_data[10][2];
	
>>>>>>> .
	struct device *dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;
	struct input_dev *input_dev;
	struct hrtimer timer;
	struct i2c_client *client;
<<<<<<< HEAD
	struct himax_i2c_platform_data *pdata;
	struct himax_virtual_key *button;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif

#ifdef HX_TP_PROC_FLASH_DUMP
	struct workqueue_struct *flash_wq;
	struct work_struct flash_work;
#endif

#ifdef HX_RST_PIN_FUNC
	int rst_gpio;
=======
	struct himax_i2c_platform_data *pdata;	
	struct himax_virtual_key *button;
    struct mutex rw_lock;

#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
	struct workqueue_struct *himax_att_wq;
	struct delayed_work work_att;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
#ifdef HX_CHIP_STATUS_MONITOR
	struct workqueue_struct *himax_chip_monitor_wq;
	struct delayed_work himax_chip_monitor;
#endif
#ifdef HX_TP_PROC_FLASH_DUMP
	struct workqueue_struct 			*flash_wq;
	struct work_struct 					flash_work;
#endif

#ifdef HX_AUTO_UPDATE_FW
	struct workqueue_struct *himax_update_wq;
	struct delayed_work work_update;
>>>>>>> .
#endif

#ifdef HX_TP_PROC_DIAG
	struct workqueue_struct *himax_diag_wq;
	struct delayed_work himax_diag_delay_wrok;
#endif
<<<<<<< HEAD
#ifdef HX_SMART_WAKEUP
	uint8_t SMWP_enable;
	uint8_t gesture_cust_en[16];
	struct wakeup_source ts_SMWP_wake_lock;
	struct workqueue_struct *himax_smwp_wq;
	struct delayed_work smwp_work;
=======

#ifdef HX_SMART_WAKEUP
	uint8_t SMWP_enable;
	uint8_t gesture_cust_en[16];
	struct wake_lock ts_SMWP_wake_lock;
>>>>>>> .
#endif

#ifdef HX_HIGH_SENSE
	uint8_t HSEN_enable;
<<<<<<< HEAD
	struct workqueue_struct *himax_hsen_wq;
	struct delayed_work hsen_work;
#endif

#if defined(HX_USB_DETECT) || defined(HX_USB_DETECT2)
=======
#endif

#if defined(HX_USB_DETECT_CALLBACK)||defined(HX_USB_DETECT_GLOBAL)
>>>>>>> .
	uint8_t usb_connected;
	uint8_t *cable_config;
#endif

<<<<<<< HEAD
	/* pinctrl data */
	struct pinctrl *ts_pinctrl;
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
};

#define HX_CMD_NOP					 0x00
#define HX_CMD_SETMICROOFF			 0x35
#define HX_CMD_SETROMRDY			 0x36
#define HX_CMD_TSSLPIN				 0x80
#define HX_CMD_TSSLPOUT				 0x81
#define HX_CMD_TSSOFF				 0x82
#define HX_CMD_TSSON				 0x83
#define HX_CMD_ROE					 0x85
#define HX_CMD_RAE					 0x86
#define HX_CMD_RLE					 0x87
#define HX_CMD_CLRES				 0x88
#define HX_CMD_TSSWRESET			 0x9E
#define HX_CMD_SETDEEPSTB			 0xD7
#define HX_CMD_SET_CACHE_FUN		 0xDD
#define HX_CMD_SETIDLE				 0xF2
#define HX_CMD_SETIDLEDELAY			 0xF3
#define HX_CMD_SELFTEST_BUFFER		 0x8D
#define HX_CMD_MANUALMODE			 0x42
#define HX_CMD_FLASH_ENABLE			 0x43
#define HX_CMD_FLASH_SET_ADDRESS	 0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND	 0x47
#define HX_CMD_FLASH_WRITE_BUFFER	 0x48
#define HX_CMD_FLASH_PAGE_ERASE		 0x4D
#define HX_CMD_FLASH_SECTOR_ERASE	 0x4E
#define HX_CMD_CB					 0xCB
#define HX_CMD_EA					 0xEA
#define HX_CMD_4A					 0x4A
#define HX_CMD_4F					 0x4F
#define HX_CMD_B9					 0xB9
#define HX_CMD_76					 0x76
=======
#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)
#ifdef HX_RESEND_CMD
	struct workqueue_struct *himax_resend_cmd_wq;
	struct delayed_work resend_cmd_work;
#endif
#endif

#if defined(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)
	struct workqueue_struct *ito_test_wq;
	struct work_struct ito_test_work;
#endif

};

#ifdef HX_CHIP_STATUS_MONITOR
struct chip_monitor_data
{
	int HX_CHIP_POLLING_COUNT;
	int HX_POLLING_TIMER;//unit:sec
	int HX_POLLING_TIMES;//ex:5(timer)x2(times)=10sec(polling time)
	int HX_ON_HAND_SHAKING;//
	int HX_CHIP_MONITOR_EN;
};
#endif

>>>>>>> .

enum input_protocol_type {
	PROTOCOL_TYPE_A	= 0x00,
	PROTOCOL_TYPE_B	= 0x01,
};

#ifdef HX_HIGH_SENSE
<<<<<<< HEAD
void himax_set_HSEN_func(struct i2c_client *client, uint8_t HSEN_enable);
=======
void himax_set_HSEN_func(struct i2c_client *client,uint8_t HSEN_enable);
>>>>>>> .
#endif

#ifdef HX_SMART_WAKEUP
#define GEST_PTLG_ID_LEN	(4)
#define GEST_PTLG_HDR_LEN	(4)
#define GEST_PTLG_HDR_ID1	(0xCC)
#define GEST_PTLG_HDR_ID2	(0x44)
<<<<<<< HEAD
#define GEST_PT_MAX_NUM		(128)

#ifdef HX_GESTURE_TRACK
static int gest_pt_cnt;
static int gest_pt_x[GEST_PT_MAX_NUM];
static int gest_pt_y[GEST_PT_MAX_NUM];
static int gest_start_x, gest_start_y, gest_end_x, gest_end_y;
static int gest_width, gest_height, gest_mid_x, gest_mid_y;
static int gn_gesture_coor[16];
#endif

void himax_set_SMWP_func(struct i2c_client *client, uint8_t SMWP_enable);
=======
#define GEST_PT_MAX_NUM 	(128)

void himax_set_SMWP_func(struct i2c_client *client,uint8_t SMWP_enable);
>>>>>>> .
extern bool FAKE_POWER_KEY_SEND;

	enum gesture_event_type {
		EV_GESTURE_01 = 0x01,
		EV_GESTURE_02,
		EV_GESTURE_03,
		EV_GESTURE_04,
		EV_GESTURE_05,
		EV_GESTURE_06,
		EV_GESTURE_07,
		EV_GESTURE_08,
		EV_GESTURE_09,
		EV_GESTURE_10,
		EV_GESTURE_11,
		EV_GESTURE_12,
		EV_GESTURE_13,
		EV_GESTURE_14,
		EV_GESTURE_15,
		EV_GESTURE_PWR = 0x80,
	};

#define KEY_CUST_01 251
#define KEY_CUST_02 252
#define KEY_CUST_03 253
#define KEY_CUST_04 254
#define KEY_CUST_05 255
#define KEY_CUST_06 256
#define KEY_CUST_07 257
#define KEY_CUST_08 258
#define KEY_CUST_09 259
#define KEY_CUST_10 260
#define KEY_CUST_11 261
#define KEY_CUST_12 262
#define KEY_CUST_13 263
#define KEY_CUST_14 264
#define KEY_CUST_15 265
#endif

<<<<<<< HEAD
#ifdef HX_ESD_WORKAROUND
	extern	u8 HX_ESD_RESET_ACTIVATE;
=======
#if defined(CONFIG_TOUCHSCREEN_HIMAX_ITO_TEST)
extern uint8_t himax_ito_test(void);
>>>>>>> .
#endif

extern int irq_enable_count;

<<<<<<< HEAD
=======
//void himax_HW_reset(uint8_t loadconfig,uint8_t int_off);

>>>>>>> .
#ifdef QCT
irqreturn_t himax_ts_thread(int irq, void *ptr);
int himax_input_register(struct himax_ts_data *ts);
#endif

<<<<<<< HEAD
int himax_chip_common_probe(struct i2c_client *client,
const struct i2c_device_id *id);
int himax_chip_common_remove(struct i2c_client *client);
int himax_chip_common_suspend(struct himax_ts_data *ts);
int himax_chip_common_resume(struct himax_ts_data *ts);
int himax_loadSensorConfig(struct i2c_client *client,
struct himax_i2c_platform_data *pdata);

#ifdef HX_USB_DETECT2
/*extern kal_bool upmu_is_chr_det(void);*/
void himax_cable_detect_func(void);
#endif

#ifdef HX_AUTO_UPDATE_FW
extern unsigned long	FW_VER_MAJ_FLASH_ADDR;
extern unsigned long	FW_VER_MIN_FLASH_ADDR;
extern unsigned long	CFG_VER_MAJ_FLASH_ADDR;
extern unsigned long	CFG_VER_MIN_FLASH_ADDR;
#endif
extern unsigned long	FW_VER_MAJ_FLASH_LENG;
extern unsigned long	FW_VER_MIN_FLASH_LENG;
extern unsigned long	CFG_VER_MAJ_FLASH_LENG;
extern unsigned long	CFG_VER_MIN_FLASH_LENG;
extern unsigned char	IC_TYPE;
extern unsigned char	IC_CHECKSUM;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
extern int himax_touch_proc_init(void);
extern void himax_touch_proc_deinit(void);
/*PROC-START*/
#ifdef HX_TP_PROC_FLASH_DUMP
extern void	himax_ts_flash_func(void);
extern void setFlashBuffer(void);
extern bool getFlashDumpGoing(void);
extern uint8_t getSysOperation(void);
extern void setSysOperation(uint8_t operation);
#endif

#ifdef HX_TP_PROC_HITOUCH
extern bool hitouch_is_connect;
#endif

#ifdef HX_TP_PROC_DIAG
	extern int touch_monitor_stop_flag;

	extern int touch_monitor_stop_limit;

	extern void	himax_ts_diag_func(void);

	extern int16_t *getMutualBuffer(void);
	extern int16_t *getMutualNewBuffer(void);
	extern int16_t *getMutualOldBuffer(void);
	extern int16_t *getSelfBuffer(void);
	extern uint8_t getXChannel(void);
	extern uint8_t getYChannel(void);
	extern uint8_t getDiagCommand(void);
	extern void setXChannel(uint8_t x);
	extern void setYChannel(uint8_t y);
	extern void setMutualBuffer(void);
	extern void setMutualNewBuffer(void);
	extern void setMutualOldBuffer(void);
	extern uint8_t coordinate_dump_enable;
	extern struct file *coordinate_fn;
	extern uint8_t diag_coor[128];
#ifdef HX_TP_PROC_2T2R
	extern int16_t *getMutualBuffer_2(void);
	extern uint8_t getXChannel_2(void);
	extern uint8_t getYChannel_2(void);
	extern void setXChannel_2(uint8_t x);
	extern void setYChannel_2(uint8_t y);
	extern void setMutualBuffer_2(void);
#endif
#endif
/*PROC-END*/
#endif

#ifdef HX_USB_DETECT2
	extern bool USB_Flag;
#endif
#ifdef HX_ESD_WORKAROUND
	extern void HX_report_ESD_event(void);
	unsigned char ESD_00_counter = 0;
	unsigned char ESD_00_Flag = 0;
#endif
bool himax_ts_init(struct himax_ts_data *ts);

=======
extern int himax_chip_common_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int himax_chip_common_remove(struct i2c_client *client);
extern int himax_chip_common_suspend(struct himax_ts_data *ts);
extern int himax_chip_common_resume(struct himax_ts_data *ts);
//#ifdef HX_AUTO_UPDATE_FW
extern bool fw_upgrade_complete; // false: not upgrade now
//#endif
>>>>>>> .
#endif

