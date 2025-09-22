#include "debug.h"
#include "ch32v30x_usbhs_device.h"
#include "debug.h"
#include "Internal_Flash.h"
// #include "SPI_FLASH.h"
#include "SW_UDISK.h"
// for bootloader
uint32_t buf[0x200 / 4] = { 0 };

void move_fw(uint32_t addr)
{
    for (uint32_t i = 0; i <= 0x8000; i += 0x200) {
        memcpy(buf, (void *)(addr + i), 0x200);
        IFlash_Prog_512(0x08002000 + i, buf);
        GPIO_WriteBit(GPIOC, GPIO_Pin_9, (i % 0x1000) > 0x800);
    }
    NVIC_SystemReset();
}

void short_press(void)
{
    move_fw(0x08020000);
}

void long_press(void)
{
    move_fw(0x08030000);
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

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
GPIO_InitTypeDef GPIO_InitStructure = { 0 };
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    // USART_Printf_Init(115200);
    // printf("SystemClk:%d\r\n", SystemCoreClock);
    // printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
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

    /* Enable Udisk */
    Udisk_Capability = IFLASH_UDISK_SIZE  / DEF_UDISK_SECTOR_SIZE;
    Udisk_Status |= DEF_UDISK_EN_FLAG;

#define APP_RUN_ADDR (0x08002000)
    NVIC_EnableIRQ(Software_IRQn);
    if (io_status && ((*(uint8_t *)APP_RUN_ADDR == 0x6F) || (*(uint8_t *)APP_RUN_ADDR == 0x97))) {
        NVIC_SetPendingIRQ(Software_IRQn);
    } else {
        USBHS_RCC_Init( );
        USBHS_Device_Init( ENABLE );
        while (1) {
            static uint32_t counter = 0;
            key_scan();
            Delay_Ms(1);
            GPIO_WriteBit(GPIOC, GPIO_Pin_9, counter++ % 1000 > 500);
            extern bool flash_start;
            extern uint32_t flash_timer;
            if (flash_start) {
                // printf("TIMER %d s\r\n", sys_time_ms());
                // printf("flash_timer %d s\r\n", flash_timer);
                if (sys_time_ms() - flash_timer > 3000)
                    NVIC_SystemReset();
            }
        }
    }
}