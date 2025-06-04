#ifndef __V4L2_H_
#define __V4L2_H_

#if defined(__cplusplus) || defined(c_plusplus)
extern "C"{
#endif
#include <linux/uvcvideo.h>
#include "common.h"
#include "datatype.h"
 
int v4l2_dev_init(DeviceContex_t **ctx, char *path);
int v4l2_read_header(DeviceContex_t *ctx);
int v4l2_read_packet(DeviceContex_t *ctx, Packet *pkt);
int v4l2_read_packet_end(DeviceContex_t *ctx, Packet *pkt);
int v4l2_read_close(DeviceContex_t *ctx);
void v4l2_dev_set_fmt(DeviceContex_t *ctx, int v4l2_fmt, int width, int height);
void v4l2_dev_deinit(DeviceContex_t *ctx);
int query_xu_control_init(DeviceContex_t *ctx);
int query_xucontrol_query(DeviceContex_t *ctx, uint8_t query, uint8_t unit, u_int8_t selector, void *data);
void save_file(void *buf, uint length,char type);

#define UUID_LE_AIT(a, b, c, d0, d1, d2, d3, d4, d5, d6, d7)        \
{ (a) & 0xff, ((a) >> 8) & 0xff, ((a) >> 16) & 0xff, ((a) >> 24) & 0xff, \
   (b) & 0xff, ((b) >> 8) & 0xff,                   \
   (c) & 0xff, ((c) >> 8) & 0xff,                   \
   (d0), (d1), (d2), (d3), (d4), (d5), (d6), (d7) }

#define UVC_AIT_EU1_GUID    UUID_LE_AIT(0x23E49ED0, 0x1178, 0x4f31, 0xAE, 0x52, 0xD2, 0xFB, 0x8A, 0x8D, 0x3B, 0x48)
#define UVC_CUS_EU2_GUID    UUID_LE_AIT(0x2C49D16A, 0x32B8, 0x4485, 0x3E, 0xA8, 0x64, 0x3A, 0x15, 0x23, 0x62, 0xF2)
#define UVC_VC_EXTENSION2_UNIT_ID 2
#define UVC_VC_EXTENSION1_UNIT_ID   6 //ISP


#define CUS_XU_SET_ISP 1
#define CUS_XU_GET_ISP 2
#define CUS_XU_ISP_LEN 4
#define REQUEST_IFRAME        (0x00000001)
#define CUS_UNKONW_CMD      (0xFFFFFFFF)
#define UVC_CS_EXTENSION_CUS_CONTROL            (5)
#define UVC_CS_EXTENSION_LEN 24
#define UVC_CS_EXTENSION_REGISTER_CONTROL   (1)

#if defined(__cplusplus) || defined(c_plusplus)
}
#endif
#endif//__V4L2_H_
