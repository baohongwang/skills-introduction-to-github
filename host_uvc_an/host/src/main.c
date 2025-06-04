#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include "v4l2.h"

#define RGB_EANBLE 0
/*extention uint params start*/
#define WRITE_FLAG                                (0x1)
#define READ_FLAG                               (0x0)
#define TRUE 1
#define FALSE 0

#define ASCII_COLOR_RED                          "\033[1;31m"
#define ASCII_COLOR_WHITE                        "\033[1;37m"
#define ASCII_COLOR_YELLOW                       "\033[1;33m"
#define ASCII_COLOR_BLUE                         "\033[1;36m"
#define ASCII_COLOR_GREEN                        "\033[1;32m"
#define ASCII_COLOR_END                          "\033[0m"
#define TOF_ERR(fmt, args...) ({do{if(g_DebugLevel>=TofDebugLevel_Err)\
                    {printf(ASCII_COLOR_RED"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})
#define TOF_WRN(fmt, args...) ({do{\
                    {printf(ASCII_COLOR_YELLOW"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})
#define TOF_INFO(fmt, args...) ({do{if(g_DebugLevel>=TofDebugLevel_Info)\
                    {printf(ASCII_COLOR_GREEN"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})
#define TOF_TIME(fmt, args...) ({do{if(g_DebugLevel>=TofDebugLevel_Time)\
                    {printf(ASCII_COLOR_BLUE"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})
#define TOF_TIME_CALC(fmt, args...) ({do{if((g_DebugFlag&0x01) && g_DebugLevel>=TofDebugLevel_Time)\
                    {printf(ASCII_COLOR_BLUE"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})
#define TOF_TIME_CALI(fmt, args...) ({do{if((g_DebugFlag&0x02) && g_DebugLevel>=TofDebugLevel_Time)\
                    {printf(ASCII_COLOR_BLUE"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})
#define TOF_TIME_FILTER(fmt, args...) ({do{if((g_DebugFlag&0x04) && g_DebugLevel>=TofDebugLevel_Time)\
                    {printf(ASCII_COLOR_BLUE"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})
#define TOF_DEBUG(fmt, args...) ({do{{printf(ASCII_COLOR_GREEN"[MI_TOF]%s[%d]: " fmt ASCII_COLOR_END"\n",__FUNCTION__,__LINE__,##args);}}while(0);})

#define DUMP_TOF_CENTRE_PIXEL  4

#define QUAD_NUM   4
    
#define TOF_MODULE_PACKAGE_LEN     60
#define TOF_MODULE_PACKAGE_DATA_LEN 50


#define assert(p)\
    do {\
        if(p)\
            printf("%s %d param valid %d\n", __func__, __LINE__, p);\
        else\
        {\
            printf("%s %d abort()\n", __func__, __LINE__);\
            abort();\
        }\
    } while(0)

        

typedef enum
{
    UVC_HDR_FREAM_HIGH_GAIN,
    UVC_HDR_FREAM_LOW_GAIN,
    UVC_HDR_FREAM_MIX,
    UVC_HDR_FREAM_NUM,
}uvc_hdr_frame_type_e;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} tof_point_cloud_temp;

typedef enum
{
    TOF_DEVICE_MIRROR_NONE,
    TOF_DEVICE_MIRROR_H,
    TOF_DEVICE_MIRROR_V,
    TOF_DEVICE_MIRROR_HV,
    TOF_DEVICE_MIRROR_NUM,
}tof_device_module_mirror;

typedef struct
{
    uint16_t   blackLevel;
    uint16_t   ythLow;
    uint16_t   ythHigh;
    uint16_t   upperCntLimitH;
    uint16_t   upperCntLimitL;
    uint32_t   expLimitLow;
    uint32_t   expLimitHigh;
    uint32_t   expStepSlow;
    uint32_t   expStepFast;
}tof_device_module_ae_sdr_param;

typedef struct
{
    uint16_t   blackLevel;
    uint16_t   ythLow;
    uint16_t   ythHigh;
    uint16_t   upperCntLimitH;
    uint16_t   upperCntLimitL;
    uint32_t   expLimitLow;
    uint32_t   expLimitHigh;
    uint32_t   expStepSlow;
    uint32_t   expStepFast;
}tof_device_module_ae_hdr_param;

typedef enum
{
    TOF_DEVICE_ROTATION_0,
    TOF_DEVICE_ROTATION_90,
    TOF_DEVICE_ROTATION_180,
    TOF_DEVICE_ROTATION_270,
    TOF_DEVICE_ROTATION_NUM,
}tof_device_module_rotation;

typedef enum 
{
  E_INTERGRATION_TIME = 0,
  E_FRAMERATE,
  E_CALIBTRATION_TYPE,
  E_FILTER_TYPE,
  E_AE,
  E_SOC_DEBUG,
  E_HDR_FRAME_TYPE,
  E_FRAME_BUF_CONFIG,
  E_TOF_MODULE_SELECT_MODE,
  E_TOF_MODULE_DATA_PACKAGE,
  E_TOF_MODULE_FLIP,
  E_TOF_MODULE_IR_THRESHOLD,
  E_TOF_MODULE_AE_PARAM,
  E_TOF_MODULE_ROTATION,
  E_TOF_MODULE_ILLUM,
  E_TOF_MODULE_EEPROM,
}package_type;

typedef enum
{
    UVC_CALIBRATION_LENS =                    1 << 0,
    UVC_CALIBRATION_NONLINEARITY =            1 << 1,
    UVC_CALIBRATION_PIXEL_WISE =              1 << 2,
    UVC_CALIBRATION_TEMPERATURE =             1 << 3,
    UVC_CALIBRATION_NUM =                     4,
} uvc_calibration_type_e;

typedef enum
{
    UVC_DEVICE_FILTER_NONE =                        0,
    UVC_DEVICE_FILTER_IIR =                         1 << 0,
    UVC_DEVICE_FILTER_DENOISE =                     1 << 1,
    UVC_DEVICE_FILTER_FLYING_PIXEL =                1 << 2,
    UVC_DEVICE_FILTER_DEALIASE_FLYPIXEL =           1 << 3,
    UVC_DEVICE_FILTER_NUM = 4,
}uvc_filter_type_e;

typedef enum
{
    UVC_DEVICE_FLIP_NONE,
    UVC_DEVICE_FLIP_H,
    UVC_DEVICE_FLIP_V,
    UVC_DEVICE_FLIP_HV,
    UVC_DEVICE_FLIP_NUM,
}uvc_flip_type_e;

typedef enum
{
    UVC_DEVICE_ROTATION_0,
    UVC_DEVICE_ROTATION_90,
    UVC_DEVICE_ROTATION_180,
    UVC_DEVICE_ROTATION_270,
    UVC_DEVICE_ROTATION_NUM,
}uvc_rotation_type_e;

typedef struct __attribute__ ((packed))
{
  uint32_t normal_time;
  uint32_t hdr_time;
}tof_device_intergration_time;

typedef struct __attribute__ ((packed))
{
    uint8_t loglevel; //0:None 1:Golden 2:INFO 3:ERROR
    int16_t x;
    int16_t y;
    uint32_t frame_num;
    uint16_t type;
}tof_device_soc_debug_config;

    
typedef struct  __attribute__ ((packed))
{
    uint32_t amplitudeSize;
    uint32_t depthSize;
    uint32_t pclSize;
    uint16_t frameHeaderSize;
    uint8_t tofNum;
}uvc_frame_size;

typedef struct __attribute__ ((packed))
{
    uint8_t useRaw;
    uint8_t resIdx;
}tof_device_mode_cfg;

typedef struct __attribute__ ((packed))
{
    uint8_t pType;
    uint8_t dSize;
    unsigned char d[TOF_MODULE_PACKAGE_DATA_LEN];
}tof_device_module_data_package;


typedef struct __attribute__ ((packed))
{
    uint16_t   blackLevel;
    uint16_t   ythLow;
    uint16_t   ythHigh;
    uint16_t   upperCntLimitH;
    uint16_t   upperCntLimitL;
    uint32_t   expLimitLow;
    uint32_t   expLimitHigh;
    uint32_t   expStepSlow;
    uint32_t   expStepFast;
}tof_device_module_ae_sdr_param_pack;

typedef struct __attribute__ ((packed))
{
    uint16_t   blackLevel;
    uint16_t   ythLow;
    uint16_t   ythHigh;
    uint16_t   upperCntLimitH;
    uint16_t   upperCntLimitL;
    uint32_t   expLimitLow;
    uint32_t   expLimitHigh;
    uint32_t   expStepSlow;
    uint32_t   expStepFast;
}tof_device_module_ae_hdr_param_pack;

typedef struct __attribute__ ((packed))
{
    uint8_t sdrValid;
    uint8_t hdrValid;
    tof_device_module_ae_sdr_param_pack sdr;
    tof_device_module_ae_hdr_param_pack hdr;
}tof_device_module_ae_param_package;

typedef struct __attribute__ ((packed))
{
    uint32_t addr;
    uint32_t size;
}ST_UVC_tof_eeprom_cfg_package_t;

typedef struct __attribute__ ((packed))
{
    uint8_t size;
    unsigned char d[TOF_MODULE_PACKAGE_DATA_LEN];
}ST_UVC_tof_eeprom_data_package_t;

typedef struct __attribute__ ((packed))
{
    uint8_t pType;
    union __attribute__ ((packed))
    {
        ST_UVC_tof_eeprom_cfg_package_t cfg;
        ST_UVC_tof_eeprom_data_package_t data;
    };
}ST_UVC_tof_eeprom_param_t;
typedef struct __attribute__ ((packed))
{
    uint8_t   enable;
    uint8_t   gain ;
}ST_UVC_tof_filter_iir_params_t;

typedef struct __attribute__ ((packed))
{
    uint8_t   enable;
    uint16_t  lpf_th;
    uint16_t  flypix_threshold;
}ST_UVC_tof_filter_denoise_params_t;

typedef struct __attribute__ ((packed))
{
    uint8_t   enable;
    uint16_t  lpf_th;
    uint16_t  flypix_gradient_th;
    uint16_t  flypix_diff_sum_th;
}ST_UVC_tof_filter_flyPixel_params_t;

typedef struct __attribute__ ((packed))
{
    uint8_t  enable;
    uint32_t low_threshold;
    uint32_t high_threshold;
}ST_UVC_tof_filter_dealiaseFlyPixel_params_t;

typedef struct __attribute__ ((packed))
{
    uvc_filter_type_e type;
    union __attribute__ ((packed))
    {
        ST_UVC_tof_filter_iir_params_t iir;
        ST_UVC_tof_filter_denoise_params_t denoise;
        ST_UVC_tof_filter_flyPixel_params_t flyPixel;
        ST_UVC_tof_filter_dealiaseFlyPixel_params_t dealiaseFlyPixel;
    };
}ST_UVC_tof_filter_param_t;


typedef struct __attribute__ ((packed))
{
    package_type pkgtype;
    union __attribute__ ((packed))
    {
        tof_device_intergration_time inter_time;
        tof_device_soc_debug_config dump_config;
        uint32_t calibration_type;
        uint32_t filter_type;
        uint8_t framerate;
        uint8_t ae;
        uvc_hdr_frame_type_e hdr_frame_type;
        uvc_frame_size frame_size;
        tof_device_mode_cfg outputCfg;
        tof_device_module_data_package dPkg;
        uint8_t flip;
        uint32_t irThd;
        tof_device_module_ae_param_package aeParam;
        uint8_t rotation;
        uint8_t enableIllum;
        ST_UVC_tof_eeprom_param_t eepromParam;
        ST_UVC_tof_filter_param_t filterParam;      
    };
    uint8_t direction;
}tof_device_cus_params;

typedef struct  __attribute__ ((packed)) 
{
    uint32_t normal_integration_time;
    uint32_t hdr_integration_time;
    int16_t temperature_sensor;
    int16_t temperature_illum;
    float cx;
    float cy;
    float k1;
    float k2;
    float k3;
    float p1;
    float p2;
    float fx;
    float fy;
}tof_device_module_depth_frame_embed_info;


int rgb_uvc_enable = 1;
int uvc_enable = 1,rgb_status =1;
static bool g_bExit = FALSE;
static int type,type_value,type_value_2;
DeviceContex_t *ctx = NULL;
DeviceContex_t *ctx_rgb = NULL;

uint32_t frameheaderSize,irSrcSize,irSrcSize,depthSrcSize,pclSrcSize;
int frame_num=0;
pthread_mutex_t s_mutex;

int set_intergration_time(tof_device_intergration_time *time)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    cus_config.inter_time.normal_time = time->normal_time;
    cus_config.inter_time.hdr_time = time->hdr_time;        
    cus_config.pkgtype = E_INTERGRATION_TIME;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}
void get_intergration_time(tof_device_cus_params *cus_config)
{
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};

    
    cus_config->pkgtype = E_INTERGRATION_TIME;
    cus_config->direction = READ_FLAG;
    //memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[0]=E_INTERGRATION_TIME;
    ctrl_data[sizeof(tof_device_cus_params) - 1] = READ_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data);
    //memcpy((void*)&cus_config, (void*)&ctrl_data, sizeof(tof_device_cus_params));
    query_xucontrol_query(ctx, UVC_GET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, cus_config); 
    printf("%s   INT =%d  ns\n",__FUNCTION__,cus_config->inter_time.normal_time);
}



void get_frame_rate(tof_device_cus_params *cus_config)
{
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};

    
    cus_config->pkgtype = E_FRAMERATE;
    cus_config->direction = READ_FLAG;
    //memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[0]=E_FRAMERATE;
    ctrl_data[sizeof(tof_device_cus_params) - 1] = READ_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data);
    //memcpy((void*)&cus_config, (void*)&ctrl_data, sizeof(tof_device_cus_params));
    query_xucontrol_query(ctx, UVC_GET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, cus_config); 
    printf("%s   Fps =%d  \n",__FUNCTION__,cus_config->framerate);
}

int set_ae_status(uint8_t status)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    cus_config.pkgtype = E_AE;
    cus_config.ae = status;//0:disable  1:enable
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}
int set_frame_rate(int frame_rate)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    cus_config.pkgtype = E_FRAMERATE;
    cus_config.framerate = frame_rate;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}

void get_ae_status(tof_device_cus_params *cus_config)
{
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};

    
    cus_config->pkgtype = E_AE;
    cus_config->direction = READ_FLAG;
    //memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[0]=E_AE;
    ctrl_data[sizeof(tof_device_cus_params) - 1] = READ_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data);
    //memcpy((void*)&cus_config, (void*)&ctrl_data, sizeof(tof_device_cus_params));
    query_xucontrol_query(ctx, UVC_GET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, cus_config); 
    printf("%s   ae =%d \n",__FUNCTION__,cus_config->ae);
}


void tof_device_get_status(tof_device_cus_params *cus_config)
{
    //tof_device_cus_params cus_config;
    //memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    //uvc_frame_size frame_size = {0};
    //cus_config.frame_size=frame_size;
    
    cus_config->pkgtype = E_FRAME_BUF_CONFIG;
    cus_config->direction = READ_FLAG;
    //memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[0]=E_FRAME_BUF_CONFIG;
    ctrl_data[sizeof(tof_device_cus_params) - 1] = READ_FLAG;
    
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data);
    //memcpy((void*)&cus_config, (void*)&ctrl_data, sizeof(tof_device_cus_params));
    query_xucontrol_query(ctx, UVC_GET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, cus_config); 
    printf("==== tofNum:%d cus_config.pkgtype=%d\n",cus_config->frame_size.tofNum,cus_config->pkgtype);
    printf("==== frameHeaderSize:%d \n",cus_config->frame_size.frameHeaderSize);
    printf("==== amplitudeSize:%d \n",cus_config->frame_size.amplitudeSize);
    printf("==== depthSize:%d \n",cus_config->frame_size.depthSize);
    printf("==== pclSize:%d \n",cus_config->frame_size.pclSize);
    printf("==== tofNum:%d \n",cus_config->frame_size.tofNum);
    frameheaderSize=cus_config->frame_size.frameHeaderSize;
    irSrcSize=cus_config->frame_size.amplitudeSize;
    depthSrcSize=cus_config->frame_size.depthSize;
    pclSrcSize=cus_config->frame_size.pclSize;
}

int tof_device_module_select_mode(uint8_t useRaw, uint8_t mode)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    cus_config.pkgtype = E_TOF_MODULE_SELECT_MODE;
    cus_config.outputCfg.useRaw = useRaw;
    cus_config.outputCfg.resIdx = mode;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}

int _tof_device_module_send(char* d, uint32_t size)
{
    int ret = 0;

    uint32_t num = 0;
    uint32_t reserved = 0;
    uint32_t offset = 0;
    
    if((d==NULL)||(size==0))
    {
        printf("[%s] Invalid params\n", __FUNCTION__);
        ret = -1;
        return ret;
    }
 
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};

    num = size/TOF_MODULE_PACKAGE_DATA_LEN;
    reserved = size%TOF_MODULE_PACKAGE_DATA_LEN;
    
    cus_config.pkgtype = E_TOF_MODULE_DATA_PACKAGE;
    cus_config.dPkg.pType = 1;
    cus_config.dPkg.dSize = 0;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 

    for(int i=0; i<num; i++)
    {
        offset = i*TOF_MODULE_PACKAGE_DATA_LEN;
        cus_config.dPkg.pType = 0;
        cus_config.dPkg.dSize = TOF_MODULE_PACKAGE_DATA_LEN;
        cus_config.direction = WRITE_FLAG;
        memcpy((void*)cus_config.dPkg.d, (void*)(d+offset), TOF_MODULE_PACKAGE_DATA_LEN);
        memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
        ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
        
        query_xu_control_init(ctx);
        query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    }

    if(reserved != 0)
    {
        if(num != 0)
        {
            offset += TOF_MODULE_PACKAGE_DATA_LEN;
        }
        cus_config.dPkg.pType = 1;
        cus_config.dPkg.dSize = reserved;
        cus_config.direction = WRITE_FLAG;
        memcpy((void*)cus_config.dPkg.d, (void*)(d+offset), reserved);
        memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
        ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
        
        query_xu_control_init(ctx);
        query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    }
    return 0;
}

int tof_device_module_set_mirror(tof_device_module_mirror type)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    cus_config.pkgtype = E_TOF_MODULE_FLIP;
    cus_config.flip = type;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}

int tof_device_module_ir_threshold(uint32_t value)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    cus_config.pkgtype = E_TOF_MODULE_IR_THRESHOLD;
    cus_config.irThd = value;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}

int tof_device_module_set_ae_param(tof_device_module_ae_sdr_param* sdr, tof_device_module_ae_hdr_param* hdr)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    if(sdr != NULL)
    {
        cus_config.aeParam.sdrValid = 1;
        cus_config.aeParam.sdr.blackLevel = sdr->blackLevel;
        cus_config.aeParam.sdr.ythLow = sdr->ythLow;
        cus_config.aeParam.sdr.ythHigh = sdr->ythHigh;
        cus_config.aeParam.sdr.upperCntLimitH = sdr->upperCntLimitH;
        cus_config.aeParam.sdr.upperCntLimitL = sdr->upperCntLimitL;
        cus_config.aeParam.sdr.expLimitLow = sdr->expLimitLow;
        cus_config.aeParam.sdr.expLimitHigh = sdr->expLimitHigh;
        cus_config.aeParam.sdr.expStepSlow = sdr->expStepSlow;
        cus_config.aeParam.sdr.expStepFast = sdr->expStepFast;
    }

    if(hdr != NULL)
    {
        cus_config.aeParam.hdrValid = 1;
        cus_config.aeParam.hdr.blackLevel = hdr->blackLevel;
        cus_config.aeParam.hdr.ythLow = hdr->ythLow;
        cus_config.aeParam.hdr.ythHigh = hdr->ythHigh;
        cus_config.aeParam.hdr.upperCntLimitH = hdr->upperCntLimitH;
        cus_config.aeParam.hdr.upperCntLimitL = hdr->upperCntLimitL;
        cus_config.aeParam.hdr.expLimitLow = hdr->expLimitLow;
        cus_config.aeParam.hdr.expLimitHigh = hdr->expLimitHigh;
        cus_config.aeParam.hdr.expStepSlow = hdr->expStepSlow;
        cus_config.aeParam.hdr.expStepFast = hdr->expStepFast;
    }
   
    cus_config.pkgtype = E_TOF_MODULE_AE_PARAM;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}

int tof_device_module_set_tof_debug(tof_device_soc_debug_config* config)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    

     cus_config.dump_config.loglevel = config->loglevel;
     cus_config.dump_config.x = config->x;
     cus_config.dump_config.y = config->y;
     cus_config.dump_config.frame_num = config->frame_num;
     cus_config.dump_config.type = config->type;   
    cus_config.pkgtype = E_SOC_DEBUG;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}

void tof_device_module_get_ae_param(tof_device_cus_params *cus_config)
{
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};


    cus_config->pkgtype = E_TOF_MODULE_AE_PARAM;
    cus_config->direction = READ_FLAG;
    //memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[0]=E_TOF_MODULE_AE_PARAM;
    ctrl_data[sizeof(tof_device_cus_params) - 1] = READ_FLAG;

    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data);
    //memcpy((void*)&cus_config, (void*)&ctrl_data, sizeof(tof_device_cus_params));
    query_xucontrol_query(ctx, UVC_GET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, cus_config); 
    printf("%s   sdr.ythLow           = %d \n",__FUNCTION__,cus_config->aeParam.sdr.ythLow );
    printf("%s   sdr.ythHigh           = %d \n",__FUNCTION__,cus_config->aeParam.sdr.ythHigh );
    printf("%s   sdr.upperCntLimitH  = %d \n",__FUNCTION__,cus_config->aeParam.sdr.upperCntLimitH );
    printf("%s   sdr.upperCntLimitL  = %d \n",__FUNCTION__,cus_config->aeParam.sdr.upperCntLimitL );
    printf("%s   sdr.expLimitLow      = %d \n",__FUNCTION__,cus_config->aeParam.sdr.expLimitLow );
    printf("%s   sdr.expLimitHigh      = %d \n",__FUNCTION__,cus_config->aeParam.sdr.expLimitHigh );

    printf("%s   hdr.ythLow           = %d \n",__FUNCTION__,cus_config->aeParam.hdr.ythLow );
    printf("%s   hdr.ythHigh           = %d \n",__FUNCTION__,cus_config->aeParam.hdr.ythHigh );
    printf("%s   hdr.upperCntLimitH  = %d \n",__FUNCTION__,cus_config->aeParam.hdr.upperCntLimitH );
    printf("%s   hdr.upperCntLimitL  = %d \n",__FUNCTION__,cus_config->aeParam.hdr.upperCntLimitL );
    printf("%s   hdr.expLimitLow      = %d \n",__FUNCTION__,cus_config->aeParam.hdr.expLimitLow );
    printf("%s   hdr.expLimitHigh      = %d \n",__FUNCTION__,cus_config->aeParam.hdr.expLimitHigh );


}

int tof_device_module_set_rotation(tof_device_module_rotation type)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    cus_config.pkgtype = E_TOF_MODULE_ROTATION;
    cus_config.rotation = type;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}

int tof_device_module_enable_illum(uint8_t enable)
{
    tof_device_cus_params cus_config;
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    
    cus_config.pkgtype = E_TOF_MODULE_ILLUM;
    cus_config.enableIllum = enable;
    cus_config.direction = WRITE_FLAG;
    memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[sizeof(tof_device_cus_params) - 1] = WRITE_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data); 
    return 0;
}
void tof_device_module_get_filter_type(tof_device_cus_params *cus_config)
{
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};

    
    cus_config->pkgtype = E_FILTER_TYPE;
    cus_config->direction = READ_FLAG;
    //memcpy((void*)&ctrl_data, (void*)&cus_config, sizeof(tof_device_cus_params));
    ctrl_data[0]=E_FILTER_TYPE;
    ctrl_data[sizeof(tof_device_cus_params) - 1] = READ_FLAG;
    
    query_xu_control_init(ctx);
    query_xucontrol_query(ctx, UVC_SET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, &ctrl_data);
    //memcpy((void*)&cus_config, (void*)&ctrl_data, sizeof(tof_device_cus_params));
    query_xucontrol_query(ctx, UVC_GET_CUR, UVC_VC_EXTENSION1_UNIT_ID, UVC_CS_EXTENSION_CUS_CONTROL, cus_config); 
    printf("%s   filtertype =0x%x  \n",__FUNCTION__,cus_config->filter_type);
}



int MI_OS_GetTime(void)
{
    struct timespec t1;

    clock_gettime(CLOCK_MONOTONIC, &t1);

    return 1000000 * (t1.tv_sec) + (t1.tv_nsec) / 1000; //us
}

void dump_tof_center_pixel(uint8_t *data)
{
    int i = 0, j = 0;
    int width=240;
    int height=180;
    int center=width*height/2+width/2;

    printf("===DUMP ir===\n");

    for(i = width / 2  - DUMP_TOF_CENTRE_PIXEL; i < width / 2 + DUMP_TOF_CENTRE_PIXEL; i++)
    {
        for(j = height / 2 - DUMP_TOF_CENTRE_PIXEL; j < height / 2 + DUMP_TOF_CENTRE_PIXEL; j++)
        {
            printf("%d ", *((uint16_t *)data + j * width + i));
        }
        printf("\n");
    }

    printf("===DUMP Depth===\n");
    for(i = width / 2 - DUMP_TOF_CENTRE_PIXEL; i < width / 2 + DUMP_TOF_CENTRE_PIXEL; i++)
    {
        for(j = height / 2 - DUMP_TOF_CENTRE_PIXEL; j < height / 2 + DUMP_TOF_CENTRE_PIXEL; j++)
        {
            printf("%.4f ", (*((uint16_t *)data + irSrcSize*sizeof(uint8_t)/sizeof(uint16_t)+j * width + i))/1000.0);

        }
        printf("\n");
    }
    printf("===DUMP PCL===\n");
    for(i = 0; i < DUMP_TOF_CENTRE_PIXEL; i++)
        {
            for(j = 0; j < DUMP_TOF_CENTRE_PIXEL; ++j)
            {
                printf("%.4f ", (*((uint16_t *)data + irSrcSize*sizeof(uint8_t)/sizeof(uint16_t)+depthSrcSize*sizeof(uint8_t)/sizeof(uint16_t) + (center+i*width)*4-1+j*3))/1000.0);
            }
            printf("\n");
        }

}


void dump_depth_embed(uint8_t *data)
{
    if(data == NULL)
        return ;
    
    tof_device_module_depth_frame_embed_info  embed = {0};
    uint8_t  *data_embed = data+irSrcSize+depthSrcSize+pclSrcSize;
    uint32_t *integration_time = (uint32_t *)data_embed;
    int16_t   *temperature  = (int16_t *)(data_embed+8);
    float      *lens = (float *)(data_embed+12);
    embed.normal_integration_time  = integration_time[0];
    embed.hdr_integration_time      = integration_time[1];
    embed.temperature_sensor       = temperature[0];
      embed.temperature_illum           =  temperature[1];
      embed.cy                         =  lens[0];
      embed.cy                         =  lens[1];
      embed.k1                         =  lens[2];
      embed.k2                         =  lens[3];
      embed.k3                         =  lens[4];
      embed.p1                         =  lens[5];
      embed.p2                         =  lens[6];
      embed.fx                         =  lens[7];
      embed.fy                         =  lens[8];
    printf("%s   normal_integration_time          = %d ns \n",__FUNCTION__,integration_time[0]);
    printf("%s   hdr_integration_time           = %d ns\n",__FUNCTION__,integration_time[1] );  
    printf("%s   temperature_sensor          = %d \n",__FUNCTION__, temperature[0]);
    printf("%s   temperature_illum           = %d \n",__FUNCTION__, temperature[1] );
    printf("%s   cx           = %f \n",__FUNCTION__, lens[0] );
    printf("%s   cy           = %f \n",__FUNCTION__, lens[1]);
    printf("%s   k1           = %f \n",__FUNCTION__, lens[2] );
    printf("%s   k2           = %f \n",__FUNCTION__, lens[3]);
    printf("%s   k3           = %f \n",__FUNCTION__, lens[4]);
    printf("%s   p1           = %f\n",__FUNCTION__, lens[5]);
    printf("%s   p2           = %f\n",__FUNCTION__, lens[6]);
    printf("%s   fx            = %f\n",__FUNCTION__, lens[7]);
    printf("%s   fy            = %f\n",__FUNCTION__, lens[8]);  
}
int nv12 = 1;
int uvc_v4l2_rgb(char *name)
{
    Packet pkt;
    int ret;
    uint32_t ISP_Data = 0;
    uint32_t FrameCnt = 0;
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    int count = 0;
    uint32_t FrameInterval =0,max_FrameInterval=0,min_FrameInterval=0;
    long long time = 0;
    long long g_u64lastTime = 0;
    uint32_t frametimesum =0;
    
    v4l2_dev_init(&ctx_rgb,  name);
    if (nv12) {
        v4l2_dev_set_fmt(ctx_rgb, V4L2_PIX_FMT_NV12, 640, 480);
    } else {
        v4l2_dev_set_fmt(ctx_rgb, V4L2_PIX_FMT_MJPEG, 1280, 720);
    }
    if (v4l2_read_header(ctx_rgb))
    {
        rgb_uvc_enable = 0;
        return -1;
    }
    query_xu_control_init(ctx_rgb);
    ISP_Data = REQUEST_IFRAME;
    query_xucontrol_query(ctx_rgb, UVC_SET_CUR, UVC_VC_EXTENSION2_UNIT_ID, CUS_XU_SET_ISP, &ISP_Data);

    while(rgb_uvc_enable) {
        //pthread_mutex_lock(&s_mutex);
        if(rgb_status == 5)
          {
            //v4l2_read_close_test(ctx_rgb);
                v4l2_read_close(ctx_rgb);           
            rgb_status = 0;

        }
        else if(rgb_status ==  6)       
          {
            //v4l2_read_start_test(ctx_rgb);
            v4l2_read_header(ctx_rgb);
            sleep(1);
            rgb_status = 1;

        }
        if(rgb_status == 1)
        {
            ret = v4l2_read_packet(ctx_rgb, &pkt);
            if(ret == -EAGAIN) {
            usleep(1000);
            continue;
            }
            if(ret >=0) {
                time = MI_OS_GetTime();
                FrameInterval = time - g_u64lastTime;
                FrameCnt++;
                if(FrameCnt== 1)
                {
                max_FrameInterval = FrameInterval;
                min_FrameInterval = FrameInterval;
                }
                
                min_FrameInterval  = (FrameInterval > min_FrameInterval )?min_FrameInterval :FrameInterval;
                max_FrameInterval = (FrameInterval > max_FrameInterval)? FrameInterval:max_FrameInterval; 
                TOF_DEBUG("RGB  Frame inteval debug %d \n",FrameInterval/1000);
                if(FrameCnt < 2000)
                {
                    frametimesum += FrameInterval;           
                    if(FrameCnt%60 == 0)
                    {
                     printf("%s: Get Pkt size %d\n", name, pkt.size);
                    TOF_DEBUG("d-thread get rgb  ok avg-fps %.2f  (min %d max %d)   FrameCnt=%d frametimesum=%d  \n\n", 1000.0/(frametimesum*1.0/FrameCnt/1000.0),min_FrameInterval/1000,max_FrameInterval/1000,FrameCnt,frametimesum);         
                        FrameCnt = 0;
                        frametimesum = 0;
                    }
                }
                g_u64lastTime = MI_OS_GetTime();
                 v4l2_read_packet_end(ctx_rgb, &pkt);
            } else {
            rgb_uvc_enable = 0;
            }
        }
        // if (++count >= 2) {
        //     count = 0;
        //     break;
        // }
        // pthread_mutex_unlock(&s_mutex);
    }

    v4l2_read_close(ctx_rgb);
    v4l2_dev_deinit(ctx_rgb);
// }



    return 0;
}

int uvc_v4l2(char *name)
{
    //DeviceContex_t *ctx = NULL;
    Packet pkt;
    int ret;
    uint32_t ISP_Data = 0;
    uint32_t FrameCnt = 0;
    uint32_t FrameInterval =0,max_FrameInterval=0,min_FrameInterval=0;
    uint32_t frametimesum =0;
    long long time = 0;
    long long g_u64lastTime = 0;
    uint8_t  now_fps =0;
    uint8_t ae_status = 0;
    tof_device_cus_params cus_config;
    
    v4l2_dev_init(&ctx,  name);
    char ctrl_data[sizeof(tof_device_cus_params)] = {0};
    v4l2_dev_set_fmt(ctx, V4L2_PIX_FMT_H264, 1280, 720);
    if (v4l2_read_header(ctx))
    {
        uvc_enable = 0;
        return -1;
    }
    query_xu_control_init(ctx);
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));
    tof_device_get_status(&cus_config);
    set_frame_rate(20);
    
    memset(&cus_config,0x0,sizeof(tof_device_cus_params));  
    get_frame_rate(&cus_config);
    now_fps = cus_config.framerate;
    TOF_DEBUG("--- before v4l2_read_packet time=%d ms  %s  framerate:%d \n", MI_OS_GetTime()/1000,name,now_fps);
    
    while(uvc_enable) {
        //pthread_mutex_lock(&s_mutex);
        ret = v4l2_read_packet(ctx, &pkt);
        if(ret == -EAGAIN) {
           usleep(10000);
           continue;
        }
        if(ret >=0) {                
        time = MI_OS_GetTime();
        FrameInterval = time - g_u64lastTime;
        FrameCnt++;
        if(FrameCnt== 1)
        {
        max_FrameInterval = FrameInterval;
        min_FrameInterval = FrameInterval;
        }

        min_FrameInterval  = (FrameInterval > min_FrameInterval )?min_FrameInterval :FrameInterval;
        max_FrameInterval = (FrameInterval > max_FrameInterval)? FrameInterval:max_FrameInterval; 
        TOF_DEBUG("TOF Frame inteval debug %d \n",FrameInterval/1000);
            if(FrameCnt < 2000)
            {
                frametimesum += FrameInterval;           
                if(FrameCnt%60 == 0)
                {
                 printf("%s: Get Pkt size %d\n", name, pkt.size);
            memset(&cus_config,0x0,sizeof(tof_device_cus_params));
            //get_intergration_time(&cus_config);   
                    TOF_DEBUG("d-thread get depth ok avg-fps %.2f  (min %d max %d)  FrameCnt=%d frametimesum=%d  cus_config.inter_time.normal_time =%d ,set fps:%d \n\n", 1000.0/(frametimesum*1.0/FrameCnt/1000.0),min_FrameInterval/1000,max_FrameInterval/1000,FrameCnt,frametimesum,cus_config.inter_time.normal_time,now_fps);
            //dump_depth_embed(pkt.data);                   
                    FrameCnt = 0;
                    frametimesum = 0;
                }
            }
            g_u64lastTime = MI_OS_GetTime();
         v4l2_read_packet_end(ctx, &pkt);

        } else {
           uvc_enable = 0;
       }
       //pthread_mutex_unlock(&s_mutex);
    }
    v4l2_read_close(ctx);
    v4l2_dev_deinit(ctx);

    return 0;
}

int main(int argc, char *argv[])
{
#if RGB_EANBLE
    char rgb_dev[20] = "/dev/video0";
    char device[20] = "/dev/video2";
#else
    char device[20] = "/dev/video0";
#endif
    pthread_t rgb_pid, pid;
    int result;

    sprintf(device, "/dev/video%d", atoi(argv[argc-2]));
    printf("tof device is %s \n",device);
    pthread_mutex_init(&s_mutex, NULL);

#if RGB_EANBLE
    nv12 =   atoi(argv[argc-1]);
    printf("rgb device is %s ,nv12 :%d \n",rgb_dev,nv12);

    pthread_create(&rgb_pid, NULL, (void *)*uvc_v4l2_rgb, rgb_dev);
#endif
    pthread_create(&pid, NULL, (void *)*uvc_v4l2, device);

    while (!g_bExit)
    {
         unsigned int u32Select = 0xff;
        int value;
        printf("select 0: set intergrationtime \n");
        printf("select 1: set frame rate \n");
        printf("select 2: set calibrantion type\n");
        printf("select 3: set sensor filter\n");
        printf("select 4: set AE status\n");
        printf("select 5: rgb stream off\n");
        printf("select 6: rgb stream on\n");
        printf("select 7: enable tof log \n");  
        printf("select 8:keep rgb connecting, but not get data    \n");             
        printf("select 9: disable AE    \n");   
        printf("select 10: enable AE    \n");           
        printf("select 13: exit\n");
        scanf("%d", &u32Select);

        if(u32Select == 0)
        {
            type=E_INTERGRATION_TIME;
            printf("\t intergrationtime value:\n");
            printf("\t SDR example: 1~1500\n");
            scanf("%d", &value);
            type_value=value;
            printf("\t HDR example: 1~1500\n");
            scanf("%d", &value);
            type_value_2=value;
        }      
        else if(u32Select == 1)
        {
            type=E_FRAMERATE;
            printf("\t frame rate value:\n");
            printf("\t example: 1~30\n");
            scanf("%d", &value);
            type_value=value;
            printf("type=%d value=%d\n",type,value);
        }
        else if(u32Select == 2)
        {
            type=E_CALIBTRATION_TYPE;
            printf("\t calibrantion value:\n");
            printf("\t example: 0  or 1\n");
            scanf("%d", &value);
            type_value=value;
        }
        else if(u32Select == 3)
        {
            type=E_FILTER_TYPE;
            printf("\t filter value:\n");
            printf("\t example: 0/1/2/4/8\n");
            scanf("%d", &value);
            type_value=value;
        }
        else if(u32Select == 4)
        {
            type=E_AE;
            printf("\t filter value:\n");
            printf("\t example: 0  or 1\n");
            scanf("%d", &value);
            type_value=value;
            printf("type=%d value=%d\n",type,value);
        }
        else if(u32Select == 5)
        {
            printf(" rgb stream off ");
            rgb_status =5 ;

        }   
        else if(u32Select == 6)
        {
            printf("rgb stream on ");
            rgb_status = 6;

        }
        else if(u32Select == 7)
        {
            printf("enable tof log ");
        tof_device_soc_debug_config config;
        memset(&config,0x0,sizeof(tof_device_soc_debug_config));    
        config.type =0 ;
        config.loglevel = 2; //:Err 1:Warn 2:Time 4:Info 
        tof_device_module_set_tof_debug(&config);

        }   
        else if(u32Select == 8)
        {
        printf("keep rgb connecting, but not get data\n ");
        rgb_status = 8;

        }   
        else if(u32Select == 9)
        {
            printf("disabel ae and set intergration time to 500 us\n ");
        set_ae_status(0);
        sleep(1);
        tof_device_intergration_time inter_time;
        inter_time.normal_time = 500*1000;
        inter_time.hdr_time = 0;
        set_intergration_time(&inter_time);
        }
        else if(u32Select == 10)
        {
            printf("disabel ae and set intergration time to 500 us\n ");
        set_ae_status(1);
        }           
        else if(u32Select == 13)
        {
            g_bExit = TRUE;
        }
        usleep(10000);
    }
    rgb_uvc_enable = 0;
    uvc_enable = 0;
    pthread_join(rgb_pid, NULL);
    pthread_join(pid, NULL);

    return 0;
}
