#include "dap_main.h"
#include "debug.h"
#include "usb2uart.h"
#include "usbd_core.h"

extern bool flash_start;
extern uint32_t flash_timer;

void enable_power_output(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOA, GPIO_Pin_5);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

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
// #define IO_CLR() GPIOB->BCR = PIN_TDI;__NOP();__NOP();__NOP();GPIOB->BSHR = PIN_TDI;__NOP();__NOP();__NOP();
// // #define IO_CLR() __NOP();
// #define IO_CLR5() IO_CLR();IO_CLR();IO_CLR();IO_CLR();IO_CLR();
// #define IO_CLR25() IO_CLR5();IO_CLR5();IO_CLR5();IO_CLR5();IO_CLR5();
// #define IO_CLR100() IO_CLR25();IO_CLR25();IO_CLR25();IO_CLR25();
// #define IO_CLR500() IO_CLR100();IO_CLR100();IO_CLR100();IO_CLR100();IO_CLR100();
// #define IO_CLR1000() IO_CLR500();IO_CLR500();
// #define IO_CLR10000() IO_CLR1000();IO_CLR1000();IO_CLR1000();IO_CLR1000();IO_CLR1000();IO_CLR1000();IO_CLR1000();IO_CLR1000();IO_CLR1000();IO_CLR1000();
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();

#if 0
    // if ((BKP->DATAR42 != 0x5AFE)&&(*(uint8_t *)0x2000 == 0x6F)) {
    if ((BKP->DATAR42 != 0x5AFE)) {
        ((int (*)(void))0x2000)();
    }

    extern void dfu_flash_init(uint8_t busid, uintptr_t reg_base);
	dfu_flash_init(0, 0);
    for(;;);
#else
    Delay_Init();  
    USART_Printf_Init(115200); 
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID()) ; 
    printf("ChipID:%08x\r\n", *(uint16_t *)0x2002) ; 

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_BKP | RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    BKP->DATAR42 = 0x5AFF;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    enable_power_output();

    uartx_preinit();
    chry_dap_init(0,0);
    while (!usb_device_is_configured(0)) {
    }

    while (1) {
        chry_dap_handle();
        chry_dap_usb2uart_handle();
    }
#endif
}

uint8_t *dfu_read_flash(uint8_t *src, uint8_t *dest, uint32_t len)
{
  uint32_t i = 0;
  uint8_t *psrc = src;

  for (i = 0; i < len; i++)
  {
    dest[i] = *psrc++;
  }
  /* Return a valid address to avoid HardFault */
  return (uint8_t *)(dest);
}

uint16_t dfu_write_flash(uint8_t *src, uint8_t *dest, uint32_t len)
{
  uint32_t addr = (uint32_t)dest;
  uint32_t i;

  addr &= 0x00FFFFFF;
  addr |= 0x08000000;
  FLASH_Unlock_Fast();
  for (i = 0; i < len; i += 256)
  {
    FLASH_ErasePage_Fast(addr);
    FLASH_ProgramPage_Fast(addr + i, (uint32_t *)(src + i));
  }
//   FLASH_Lock_Fast();
  return 0;
}

uint16_t dfu_erase_flash(uint32_t add)
{
    // add &= 0x00FFFFFF;
    // add |= 0x08000000;
    // FLASH_Unlock_Fast();
    // FLASH_ErasePage_Fast(add);
    // FLASH_Lock_Fast();
    return 0;
}

void dfu_leave(void)
{
    NVIC_SystemReset();
}