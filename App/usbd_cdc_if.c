/*
 * USB CDC 应用层实现: 收到的字节存进环形缓冲区(usb_cdc_getc取), 发送走非阻塞Transmit。
 * 不做任何UART桥接 —— 这路USB就是命令通道本身, 跟uart_log/uart_rx_getc是并列的第二路。
 */
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

static uint8_t UserRxBufferFS[CDC_DATA_FS_MAX_PACKET_SIZE];

#define USB_RXRING_SIZE 128
static volatile uint8_t  rx_ring[USB_RXRING_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0;

static int8_t CDC_Init_FS(void)
{
   USBD_CDC_SetTxBuffer(&hUsbDeviceFS, NULL, 0);
   USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
   return USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{ return USBD_OK; }

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
   (void)cmd; (void)pbuf; (void)length;
   return USBD_OK;   /* 波特率/流控这些线路参数是给虚拟串口摆样子的, 这里不需要真处理 */
}

static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len)
{
   for (uint32_t i = 0; i < *Len; i++) {
      uint16_t next = (uint16_t)((rx_head + 1) % USB_RXRING_SIZE);
      if (next != rx_tail) {          /* 满了就丢, 跟uart_rx一致的策略 */
         rx_ring[rx_head] = Buf[i];
         rx_head = next;
      }
   }
   USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &UserRxBufferFS[0]);
   USBD_CDC_ReceivePacket(&hUsbDeviceFS);
   return USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
   (void)Buf; (void)Len; (void)epnum;
   return USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
   CDC_Init_FS,
   CDC_DeInit_FS,
   CDC_Control_FS,
   CDC_Receive_FS,
   CDC_TransmitCplt_FS
};

int usb_cdc_getc(void)
{
   if (rx_head == rx_tail) return -1;
   uint8_t c = rx_ring[rx_tail];
   rx_tail = (uint16_t)((rx_tail + 1) % USB_RXRING_SIZE);
   return (int)c;
}

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
   USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;

   if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return USBD_FAIL;
   if (hcdc == NULL || hcdc->TxState != 0) return USBD_BUSY;   /* 上一包还没发完, 直接丢弃这次 */

   USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
   return USBD_CDC_TransmitPacket(&hUsbDeviceFS);
}
