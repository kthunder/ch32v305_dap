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
uint32_t buf[256 / 4] = { 0 };
// for bootloader
void short_press(void)
{
    for (uint32_t i = 0; i <= 0x10000; i += 0x100) {
        memcpy(buf, (void *)(0x08020000 + i), 256);
        FLASH_ROM_ERASE(0x08002000 + i, 0x100);
        FLASH_ROM_WRITE(0x08002000 + i, buf, 0x100);
        GPIO_WriteBit(GPIOC, GPIO_Pin_9, (i % 0x1000) > 0x800);
    }
    NVIC_SystemReset();
}

void long_press(void)
{
    for (uint32_t i = 0; i <= 0x10000; i += 0x100) {
        memcpy(buf, (void *)(0x08030000 + i), 256);
        FLASH_ROM_ERASE(0x08002000 + i, 0x100);
        FLASH_ROM_WRITE(0x08002000 + i, buf, 0x100);
        GPIO_WriteBit(GPIOC, GPIO_Pin_9, (i % 0x1000) > 0x800);
    }
    NVIC_SystemReset();
}

#define THRESHOLD 10

static void key_scan(void)
{
    static uint32_t pressed_ms = 0;
    // printf("pin %d : %d\r\n", ipin, gpio_get(ipin));
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == 0) {
        pressed_ms++;
    } else {
        if (pressed_ms > (2000)) {
            long_press();
        } else if (pressed_ms > 20) {
            short_press();
        }
        pressed_ms = 0;
    }
}

#define APP_RUN_ADDR (0x08002000)
extern void msc_ram_init(uint8_t busid, uintptr_t reg_base);

void check_iap_status(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    volatile uint8_t io_status = 0;
    for (uint8_t i = 0; i < 10; i++) {
        io_status += GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7);
    }

    while (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) == 0)
        ;

    NVIC_EnableIRQ(Software_IRQn);
    if (io_status && (*(uint8_t *)APP_RUN_ADDR == 0x6F)) {
        NVIC_SetPendingIRQ(Software_IRQn);
    } else {
        // msc_ram_init(0, USBHS_BASE);
        while (1) {
            static uint32_t counter = 0;
            key_scan();
            Delay_Ms(1);
            GPIO_WriteBit(GPIOC, GPIO_Pin_9, counter++ % 1000 > 500);
            // if (flash_start) {
            //     // printf("TIMER %d s\r\n", sys_time_ms());
            //     // printf("flash_timer %d s\r\n", flash_timer);
            //     if (sys_time_ms() - flash_timer > 3000)
            //         NVIC_SystemReset();
            // }
        }
    }
}

void SW_Handler(void)
{
    __asm("li  a6, 0x2000");
    __asm("jr  a6");
}

#define APP  0
#define BOOT 1

#ifndef PROJ
#define PROJ BOOT
#endif
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
    Delay_Init();
    // USART_Printf_Init(115200);
    // printf("SystemClk:%d\r\n",SystemCoreClock);
    // printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
#if PROJ == APP
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
    enable_power_output();

    uartx_preinit();
    chry_dap_init(0, USBHS_BASE);
    while (!usb_device_is_configured(0)) {
    }

    while (1) {
        chry_dap_handle();
        chry_dap_usb2uart_handle();
    }
#elif PROJ == BOOT
    void check_iap_status(void);
    check_iap_status();
#endif
}