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
    Delay_Init();
    // USART_Printf_Init(115200);
    // printf("SystemClk:%d\r\n", SystemCoreClock);
    // printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
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
}