/*
 * USB Device 底层驱动胶水层(PCD HAL <-> USBD Core), 只用 OTG_FS。
 *
 * 本板"USB Slave"(Micro-USB)只接了 PA11(DM)/PA12(DP) 两根数据线;
 * VBUS感知(PA9)和ID脚(PA10)在本板上已经被USART1(接CH340)占用,
 * 没有额外接到OTG_FS的VBUS/ID功能上 —— 所以这里绝不能去初始化PA9/PA10,
 * 否则会把好不容易能用的USART1_TX/RX冲掉。改用 vbus_sensing_enable=0,
 * 让PCD直接认为"电一直有", 不去采样VBUS引脚。
 */
#include "stm32f4xx_hal.h"
#include "usbd_core.h"

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* ============== PCD MSP (时钟/GPIO/NVIC) ============== */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
   GPIO_InitTypeDef g = {0};

   if (hpcd->Instance != USB_OTG_FS) return;

   __HAL_RCC_GPIOA_CLK_ENABLE();

   /* PA11=DM, PA12=DP, 复用功能, 不接VBUS/ID(见上方注释) */
   g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
   g.Mode      = GPIO_MODE_AF_PP;
   g.Pull      = GPIO_NOPULL;
   g.Speed     = GPIO_SPEED_FREQ_HIGH;
   g.Alternate = GPIO_AF10_OTG_FS;
   HAL_GPIO_Init(GPIOA, &g);

   __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

   HAL_NVIC_SetPriority(OTG_FS_IRQn, 6, 0);
   HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
   if (hpcd->Instance != USB_OTG_FS) return;
   __HAL_RCC_USB_OTG_FS_CLK_DISABLE();
   HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
}

/* ============== PCD -> USBD Core 回调 ============== */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{ USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup); }

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{ USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff); }

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{ USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff); }

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{ USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData); }

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
   USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
   USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{ USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData); }

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{ USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData); }

void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{ USBD_LL_IsoOUTIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum); }

void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{ USBD_LL_IsoINIncomplete((USBD_HandleTypeDef *)hpcd->pData, epnum); }

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{ USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData); }

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{ USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData); }

/* ============== USBD Core -> PCD (LL接口) ============== */
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
   hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
   hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
   hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
   hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
   hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
   hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
   hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
   hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
   hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;  /* PA9未接VBUS, 见文件头注释 */
   hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;

   hpcd_USB_OTG_FS.pData = pdev;
   pdev->pData = &hpcd_USB_OTG_FS;

   if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) return USBD_FAIL;

   HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);
   HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);
   HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);

   return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{ HAL_PCD_DeInit(pdev->pData); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{ HAL_PCD_Start(pdev->pData); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{ HAL_PCD_Stop(pdev->pData); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{ HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{ HAL_PCD_EP_Close(pdev->pData, ep_addr); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{ HAL_PCD_EP_Flush(pdev->pData, ep_addr); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{ HAL_PCD_EP_SetStall(pdev->pData, ep_addr); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{ HAL_PCD_EP_ClrStall(pdev->pData, ep_addr); return USBD_OK; }

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
   PCD_HandleTypeDef *hpcd = pdev->pData;
   if ((ep_addr & 0x80) == 0x80) return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
   return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{ HAL_PCD_SetAddress(pdev->pData, dev_addr); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{ HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size); return USBD_OK; }

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{ HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size); return USBD_OK; }

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{ return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr); }

void USBD_LL_Delay(uint32_t Delay)
{ HAL_Delay(Delay); }
