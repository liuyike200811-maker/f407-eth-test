/*
 * USB 设备描述符(仿ST官方usbd_desc.h裁剪)
 */
#ifndef __USBD_DESC_H
#define __USBD_DESC_H

#include "usbd_def.h"

#define DEVICE_ID1   (0x1FFF7A10)
#define DEVICE_ID2   (0x1FFF7A14)
#define DEVICE_ID3   (0x1FFF7A18)

#define USB_SIZ_STRING_SERIAL   0x1AU

extern USBD_DescriptorsTypeDef FS_Desc;

#endif /* __USBD_DESC_H */
