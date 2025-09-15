#include "debug.h"

#define SECTION_DATA(sec) __attribute__((section(sec)))

#define FUNC_ALIAS(func_from, func_to) void func_to(void) __attribute__((alias(#func_from), weak))

__weak_symbol void Default_Handler(void)
{
    while (1)
        ;
}
FUNC_ALIAS(Default_Handler, NMI_Handler); /* NMI */
FUNC_ALIAS(Default_Handler, HardFault_Handler);
FUNC_ALIAS(Default_Handler, Ecall_M_Mode_Handler);
FUNC_ALIAS(Default_Handler, Ecall_U_Mode_Handler);
FUNC_ALIAS(Default_Handler, Break_Point_Handler);
FUNC_ALIAS(Default_Handler, SysTick_Handler);
FUNC_ALIAS(Default_Handler, SW_Handler);
FUNC_ALIAS(Default_Handler, WWDG_IRQHandler);
FUNC_ALIAS(Default_Handler, PVD_IRQHandler);
FUNC_ALIAS(Default_Handler, TAMPER_IRQHandler);
FUNC_ALIAS(Default_Handler, RTC_IRQHandler);
FUNC_ALIAS(Default_Handler, FLASH_IRQHandler);
FUNC_ALIAS(Default_Handler, RCC_IRQHandler);
FUNC_ALIAS(Default_Handler, EXTI0_IRQHandler);
FUNC_ALIAS(Default_Handler, EXTI1_IRQHandler);
FUNC_ALIAS(Default_Handler, EXTI2_IRQHandler);
FUNC_ALIAS(Default_Handler, EXTI3_IRQHandler);
FUNC_ALIAS(Default_Handler, EXTI4_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA1_Channel1_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA1_Channel2_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA1_Channel3_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA1_Channel4_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA1_Channel5_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA1_Channel6_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA1_Channel7_IRQHandler);
FUNC_ALIAS(Default_Handler, ADC1_2_IRQHandler);
FUNC_ALIAS(Default_Handler, USB_HP_CAN1_TX_IRQHandler);
FUNC_ALIAS(Default_Handler, USB_LP_CAN1_RX0_IRQHandler);
FUNC_ALIAS(Default_Handler, CAN1_RX1_IRQHandler);
FUNC_ALIAS(Default_Handler, CAN1_SCE_IRQHandler);
FUNC_ALIAS(Default_Handler, EXTI9_5_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM1_BRK_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM1_UP_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM1_TRG_COM_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM1_CC_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM2_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM3_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM4_IRQHandler);
FUNC_ALIAS(Default_Handler, I2C1_EV_IRQHandler);
FUNC_ALIAS(Default_Handler, I2C1_ER_IRQHandler);
FUNC_ALIAS(Default_Handler, I2C2_EV_IRQHandler);
FUNC_ALIAS(Default_Handler, I2C2_ER_IRQHandler);
FUNC_ALIAS(Default_Handler, SPI1_IRQHandler);
FUNC_ALIAS(Default_Handler, SPI2_IRQHandler);
FUNC_ALIAS(Default_Handler, USART1_IRQHandler);
FUNC_ALIAS(Default_Handler, USART2_IRQHandler);
FUNC_ALIAS(Default_Handler, USART3_IRQHandler);
FUNC_ALIAS(Default_Handler, EXTI15_10_IRQHandler);
FUNC_ALIAS(Default_Handler, RTCAlarm_IRQHandler);
FUNC_ALIAS(Default_Handler, USBWakeUp_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM8_BRK_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM8_UP_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM8_TRG_COM_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM8_CC_IRQHandler);
FUNC_ALIAS(Default_Handler, RNG_IRQHandler);
FUNC_ALIAS(Default_Handler, SDIO_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM5_IRQHandler);
FUNC_ALIAS(Default_Handler, SPI3_IRQHandler);
FUNC_ALIAS(Default_Handler, UART4_IRQHandler);
FUNC_ALIAS(Default_Handler, UART5_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM6_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM7_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel1_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel2_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel3_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel4_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel5_IRQHandler);
FUNC_ALIAS(Default_Handler, ETH_IRQHandler);
FUNC_ALIAS(Default_Handler, ETH_WKUP_IRQHandler);
FUNC_ALIAS(Default_Handler, CAN2_TX_IRQHandler);
FUNC_ALIAS(Default_Handler, CAN2_RX0_IRQHandler);
FUNC_ALIAS(Default_Handler, CAN2_RX1_IRQHandler);
FUNC_ALIAS(Default_Handler, CAN2_SCE_IRQHandler);
FUNC_ALIAS(Default_Handler, USBFS_IRQHandler);
FUNC_ALIAS(Default_Handler, USBHSWakeup_IRQHandler);
FUNC_ALIAS(Default_Handler, USBHS_IRQHandler);
FUNC_ALIAS(Default_Handler, DVP_IRQHandler);
FUNC_ALIAS(Default_Handler, UART6_IRQHandler);
FUNC_ALIAS(Default_Handler, UART7_IRQHandler);
FUNC_ALIAS(Default_Handler, UART8_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM9_BRK_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM9_UP_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM9_TRG_COM_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM9_CC_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM10_BRK_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM10_UP_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM10_TRG_COM_IRQHandler);
FUNC_ALIAS(Default_Handler, TIM10_CC_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel6_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel7_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel8_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel9_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel10_IRQHandler);
FUNC_ALIAS(Default_Handler, DMA2_Channel11_IRQHandler);
void Reset_Handler();

SECTION_DATA(".vector")
void (*vector[128])(void) = {
    Reset_Handler,
    0,
    NMI_Handler,       /* NMI */
    HardFault_Handler, /* Hard Fault */
    0,
    Ecall_M_Mode_Handler, /* Ecall M Mode */
    0,
    0,
    Ecall_U_Mode_Handler, /* Ecall U Mode */
    Break_Point_Handler,  /* Break Point */
    0,
    0,
    SysTick_Handler, /* SysTick */
    0,
    SW_Handler, /* SW */
    0,
    /* external Interrupts */
    WWDG_IRQHandler,            /* Window Watchdog */
    PVD_IRQHandler,             /* PVD through EXTI Line detect */
    TAMPER_IRQHandler,          /* TAMPER */
    RTC_IRQHandler,             /* RTC */
    FLASH_IRQHandler,           /* Flash */
    RCC_IRQHandler,             /* RCC */
    EXTI0_IRQHandler,           /* EXTI Line 0 */
    EXTI1_IRQHandler,           /* EXTI Line 1 */
    EXTI2_IRQHandler,           /* EXTI Line 2 */
    EXTI3_IRQHandler,           /* EXTI Line 3 */
    EXTI4_IRQHandler,           /* EXTI Line 4 */
    DMA1_Channel1_IRQHandler,   /* DMA1 Channel 1 */
    DMA1_Channel2_IRQHandler,   /* DMA1 Channel 2 */
    DMA1_Channel3_IRQHandler,   /* DMA1 Channel 3 */
    DMA1_Channel4_IRQHandler,   /* DMA1 Channel 4 */
    DMA1_Channel5_IRQHandler,   /* DMA1 Channel 5 */
    DMA1_Channel6_IRQHandler,   /* DMA1 Channel 6 */
    DMA1_Channel7_IRQHandler,   /* DMA1 Channel 7 */
    ADC1_2_IRQHandler,          /* ADC1_2 */
    USB_HP_CAN1_TX_IRQHandler,  /* USB HP and CAN1 TX */
    USB_LP_CAN1_RX0_IRQHandler, /* USB LP and CAN1RX0 */
    CAN1_RX1_IRQHandler,        /* CAN1 RX1 */
    CAN1_SCE_IRQHandler,        /* CAN1 SCE */
    EXTI9_5_IRQHandler,         /* EXTI Line 9..5 */
    TIM1_BRK_IRQHandler,        /* TIM1 Break */
    TIM1_UP_IRQHandler,         /* TIM1 Update */
    TIM1_TRG_COM_IRQHandler,    /* TIM1 Trigger and Commutation */
    TIM1_CC_IRQHandler,         /* TIM1 Capture Compare */
    TIM2_IRQHandler,            /* TIM2 */
    TIM3_IRQHandler,            /* TIM3 */
    TIM4_IRQHandler,            /* TIM4 */
    I2C1_EV_IRQHandler,         /* I2C1 Event */
    I2C1_ER_IRQHandler,         /* I2C1 Error */
    I2C2_EV_IRQHandler,         /* I2C2 Event */
    I2C2_ER_IRQHandler,         /* I2C2 Error */
    SPI1_IRQHandler,            /* SPI1 */
    SPI2_IRQHandler,            /* SPI2 */
    USART1_IRQHandler,          /* USART1 */
    USART2_IRQHandler,          /* USART2 */
    USART3_IRQHandler,          /* USART3 */
    EXTI15_10_IRQHandler,       /* EXTI Line 15..10 */
    RTCAlarm_IRQHandler,        /* RTC Alarm through EXTI Line */
    USBWakeUp_IRQHandler,       /* USB Wakeup from suspend */
    TIM8_BRK_IRQHandler,        /* TIM8 Break */
    TIM8_UP_IRQHandler,         /* TIM8 Update */
    TIM8_TRG_COM_IRQHandler,    /* TIM8 Trigger and Commutation */
    TIM8_CC_IRQHandler,         /* TIM8 Capture Compare */
    RNG_IRQHandler,             /* RNG */
    0,
    SDIO_IRQHandler,           /* SDIO */
    TIM5_IRQHandler,           /* TIM5 */
    SPI3_IRQHandler,           /* SPI3 */
    UART4_IRQHandler,          /* UART4 */
    UART5_IRQHandler,          /* UART5 */
    TIM6_IRQHandler,           /* TIM6 */
    TIM7_IRQHandler,           /* TIM7 */
    DMA2_Channel1_IRQHandler,  /* DMA2 Channel 1 */
    DMA2_Channel2_IRQHandler,  /* DMA2 Channel 2 */
    DMA2_Channel3_IRQHandler,  /* DMA2 Channel 3 */
    DMA2_Channel4_IRQHandler,  /* DMA2 Channel 4 */
    DMA2_Channel5_IRQHandler,  /* DMA2 Channel 5 */
    ETH_IRQHandler,            /* ETH */
    ETH_WKUP_IRQHandler,       /* ETH WakeUp */
    CAN2_TX_IRQHandler,        /* CAN2 TX */
    CAN2_RX0_IRQHandler,       /* CAN2 RX0 */
    CAN2_RX1_IRQHandler,       /* CAN2 RX1 */
    CAN2_SCE_IRQHandler,       /* CAN2 SCE */
    USBFS_IRQHandler,          /* USBFS */
    USBHSWakeup_IRQHandler,    /* USBHS Wakeup */
    USBHS_IRQHandler,          /* USBHS */
    DVP_IRQHandler,            /* DVP */
    UART6_IRQHandler,          /* UART6 */
    UART7_IRQHandler,          /* UART7 */
    UART8_IRQHandler,          /* UART8 */
    TIM9_BRK_IRQHandler,       /* TIM9 Break */
    TIM9_UP_IRQHandler,        /* TIM9 Update */
    TIM9_TRG_COM_IRQHandler,   /* TIM9 Trigger and Commutation */
    TIM9_CC_IRQHandler,        /* TIM9 Capture Compare */
    TIM10_BRK_IRQHandler,      /* TIM10 Break */
    TIM10_UP_IRQHandler,       /* TIM10 Update */
    TIM10_TRG_COM_IRQHandler,  /* TIM10 Trigger and Commutation */
    TIM10_CC_IRQHandler,       /* TIM10 Capture Compare */
    DMA2_Channel6_IRQHandler,  /* DMA2 Channel 6 */
    DMA2_Channel7_IRQHandler,  /* DMA2 Channel 7 */
    DMA2_Channel8_IRQHandler,  /* DMA2 Channel 8 */
    DMA2_Channel9_IRQHandler,  /* DMA2 Channel 9 */
    DMA2_Channel10_IRQHandler, /* DMA2 Channel 10 */
    DMA2_Channel11_IRQHandler, /* DMA2 Channel 11 */
};

static void __memset(uint8_t *dist, uint8_t val, uint32_t len)
{
    while (len--)
        *dist++ = val;
}
static void __memcpy(uint8_t *dist, uint8_t *src, uint32_t len)
{
    while (len--)
        *dist++ = *src++;
}

SECTION_DATA(".init")
void Reset_Handler()
{
    asm volatile(
        ".option push\n"
        ".option norelax\n"
        "la gp, __global_pointer$\n"
        ".option pop" : :);
    asm volatile("la sp, _eusrstack");
    extern uint8_t _data_vma[], _data_lma[], _edata[], _sbss[], _ebss[];
    __memcpy(_data_vma, _data_lma, _edata - _data_vma);
    __memset(_sbss, 0, _ebss - _sbss);
    // asm volatile("csrw 0xbc0, %0" : : "r"(0x1f));
    // asm volatile("csrw 0x804, %0" : : "r"(0x0b));
    // asm volatile("csrw mstatus, %0" : : "r"(0x6088));
    // __set_MTVEC((uint32_t)vector);
    /* Configure pipelining and instruction prediction */
    asm volatile("li t0, 0x1f");
    asm volatile("csrw 0xbc0, t0");
    /* Enable interrupt nesting and hardware stack */
    asm volatile("li t0, 0x0b");
    asm volatile("csrw 0x804, t0");
    /* Enable floating point and global interrupt, configure privileged mode */
    asm volatile("li t0, 0x6088");
    asm volatile("csrw mstatus, t0");
    /* Configure the interrupt vector table recognition mode and entry address mode */
    asm volatile("la t0, vector");
    asm volatile("ori t0, t0, 3");
    asm volatile("csrw mtvec, t0");
    SystemInit();
    extern int main(void);
    __set_MEPC((uint32_t)main);
    asm volatile("mret");
}
