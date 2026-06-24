#include "debug.h"
#include "usb2uart.h"
#include "usbd_core.h"

void USBHS_RCC_Init(void)
{
    RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
    RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
    RCC_USBHSConfig(RCC_USBPLL_Div3);
    RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
    RCC_USBHSPHYPLLALIVEcmd(ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
}

void usb_dc_low_level_init(void)
{
    USBHS_RCC_Init();
    NVIC_EnableIRQ(USBHS_IRQn);
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();

    if ((BKP->DATAR42 != 0x5AFE)&&(*(uint8_t *)0x2000 == 0x6F)) {
    // if ((BKP->DATAR42 != 0x5AFE)) {
        ((int (*)(void))0x2000)();
    }

    RCC->APB1PCENR |= RCC_APB1Periph_BKP | RCC_APB1Periph_PWR;
    PWR->CTLR |= (1 << 8);

    FLASH_Unlock_Fast();
    extern void dfu_flash_init(uint8_t busid, uintptr_t reg_base);
	  dfu_flash_init(0, 0);
    for(;;);
}

uint8_t *dfu_read_flash(uint8_t *src, uint8_t *dest, uint32_t len)
{
  memcpy(dest, src, len);
  /* Return a valid address to avoid HardFault */
  return (uint8_t *)(dest);
}

uint16_t dfu_write_flash(uint8_t *src, uint8_t *dest, uint32_t len)
{
  uint32_t addr = (uint32_t)dest;
  uint32_t i;

  addr &= 0x00FFFFFF;
  addr |= 0x08000000;
  for (i = 0; i < len; i += 256)
  {
    // FLASH_ErasePage_Fast(addr);
    FLASH_ProgramPage_Fast(addr + i, (uint32_t *)(src + i));
  }
//   FLASH_Lock_Fast();
  return 0;
}

uint16_t dfu_erase_flash(uint32_t addr)
{
  FLASH_ErasePage_Fast(addr);
  return 0;
}
void dfu_leave(void){BKP->DATAR42 = 0x5AFD;NVIC_SystemReset();}
