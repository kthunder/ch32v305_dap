#include "debug.h"

// for bootloader
void short_press(void)
{
    // NVIC_SystemReset();
}

void long_press(void)
{
    // NVIC_SystemReset();
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
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    // USART_Printf_Init(115200);
    // printf("SystemClk:%d\r\n", SystemCoreClock);
    // printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());

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
    // NVIC_EnableIRQ(Software_IRQn);
    // if (io_status && ((*(uint8_t *)APP_RUN_ADDR == 0x6F) || (*(uint8_t *)APP_RUN_ADDR == 0xEF))) {
    //     NVIC_SetPendingIRQ(Software_IRQn);
    // } else {
        // msc_ram_init(0, 0);
        while (1) {
            static uint32_t counter = 0;

            // key_scan();
            Delay_Ms(1);
            // 添加USB连接状态检查
            GPIO_WriteBit(GPIOC, GPIO_Pin_9, counter++ % 1000 > 500);
            // if (flash_start) {
            //     // printf("TIMER %d s\r\n", sys_time_ms());
            //     // printf("flash_timer %d s\r\n", flash_timer);
            //     if (sys_time_ms() - flash_timer > 3000)
            //         NVIC_SystemReset();
            // }
        }
    // }
}