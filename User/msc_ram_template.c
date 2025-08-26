/*
 * Copyright (c) 2024, sakumisu
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usbd_msc.h"
#include "debug.h"

#define MSC_IN_EP  0x81
#define MSC_OUT_EP 0x02

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

#define USB_CONFIG_SIZE (9 + MSC_DESCRIPTOR_LEN)

#ifdef CONFIG_USB_HS
#define MSC_MAX_MPS 512
#else
#define MSC_MAX_MPS 64
#endif

const uint8_t msc_ram_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MSC_DESCRIPTOR_INIT(0x00, MSC_OUT_EP, MSC_IN_EP, MSC_MAX_MPS, 0x02),
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x26,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ' ', 0x00,                  /* wcChar9 */
    'M', 0x00,                  /* wcChar10 */
    'S', 0x00,                  /* wcChar11 */
    'C', 0x00,                  /* wcChar12 */
    ' ', 0x00,                  /* wcChar13 */
    'D', 0x00,                  /* wcChar14 */
    'E', 0x00,                  /* wcChar15 */
    'M', 0x00,                  /* wcChar16 */
    'O', 0x00,                  /* wcChar17 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '2', 0x00,                  /* wcChar3 */
    '1', 0x00,                  /* wcChar4 */
    '2', 0x00,                  /* wcChar5 */
    '3', 0x00,                  /* wcChar6 */
    '4', 0x00,                  /* wcChar7 */
    '5', 0x00,                  /* wcChar8 */
    '6', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x00,
    0x00,
    0x00,
    0x40,
    0x00,
    0x00,
#endif
    0x00
};

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

#define SECTOR_SIZE      512
#define TOTAL_SECTORS    2048 // 1024KB = 2048扇区
#define SECTORS_PER_FAT  4    // FAT表需要4扇区
#define ROOT_DIR_SECTORS 4    // 根目录4扇区
#define RESERVED_SECTORS 1    // 保留扇区1个
#define DATA_START_LBA   9    // 1+4+4=9

const uint8_t BootSector[] = {
    0xEB, 0x3C, 0x90,
    'M', 'S', 'D', 'O', 'S', '5', '.', '0',
    0x00, 0x02,                             // 512字节/扇区
    0x01,                                   // 1扇区/簇
    0x01, 0x00,                             // 1个保留扇区
    0x01,                                   // 1个FAT表
    0x40, 0x00,                             // 64个根目录条目
    0x00, 0x08,                             // 2048扇区(0x0800小端序)
    0xF0,                                   // 媒体描述符
    0x04, 0x00,                             // 4扇区/FAT
    0x01, 0x00,                             // 扇区/磁道
    0x01, 0x00,                             // 磁头数
    0x00, 0x00, 0x00, 0x00,                 // 隐藏扇区
    0x00, 0x00, 0x00, 0x00,                 // 大扇区数
    0x00,                                   // 驱动器号
    0x00,                                   // 保留
    0x29,                                   // 签名
    0x12, 0x34, 0x56, 0x78,                 // 卷序列号
    'U', 'S', 'B', 'F', 'L', 'A', 'S', 'H', // 卷标
    'F', 'A', 'T', '1', '2', ' ', ' ', ' ', // 文件系统类型
    // [62 ... 509] = 0x00,// Boot code (fill with 0)
    // 0x55, 0xAA // Boot sector signature(write in func)
};
void usbd_msc_get_cap(uint8_t busid, uint8_t lun, uint32_t *block_num, uint32_t *block_size)
{
    *block_num = TOTAL_SECTORS; //Pretend having so many buffer,not has actually.
    *block_size = SECTOR_SIZE;
}
// Sector 0: Boot Sector (保留扇区)
// Sector 1-2: FAT Table (FAT表，2个扇区)
// Sector 3: Root Directory (根目录)
// Sector 4+: Data Area (数据区)
int usbd_msc_sector_read(uint8_t busid, uint8_t lun, uint32_t  sector, uint8_t *buffer, uint32_t length)
{
    if (sector == 0) {
        memcpy(buffer, BootSector, sizeof(BootSector));
        buffer[510] = 0x55;
        buffer[511] = 0xAA;
    } else if (sector >= 1 && sector <= 4) { // FAT表扇区1-4
        memset(buffer, 0, SECTOR_SIZE);
        if (sector == 1) {
            buffer[0] = 0xF0;
            buffer[1] = 0xFF;
            buffer[2] = 0xFF;
        }
    } else if (sector >= 5 && sector <= 8) { // 根目录扇区5-8
        memset(buffer, 0, SECTOR_SIZE);
    } else if (sector >= 9) { // 数据区
        memset(buffer, 0x00, SECTOR_SIZE);
    }
    return 0;
}
bool flash_start = false;
uint32_t flash_timer = 0;
int usbd_msc_sector_write(uint8_t busid, uint8_t lun, uint32_t sector, uint8_t *buffer, uint32_t length)
{
//  if (sector < BLOCK_COUNT)
//      memcpy(mass_block[sector].BlockSpace, buffer, length);
    if (sector >= 12) {
        flash_start = true;
        uint32_t offset = (sector - 12) * SECTOR_SIZE;

        // uint32_t x = 0;
        // for (int i = 0; i < length; i++) {
        //     x += buffer[i];
        //     // printf("%02X ", buffer[i]);
        // }
        // // printf("\n");
#include "Internal_Flash.h"
        // if (0x08020000 + offset < 0x08040000) {
            // IFlash_Prog_512((uint32_t)(0x08002000 + offset), (uint32_t *)buffer);
            FLASH_ROM_ERASE(0x08002000 + offset, length);
            FLASH_ROM_WRITE(0x08002000 + offset, buffer, length);
            // printf("sector %03d, size %d, x %08X \r\n", sector, length, x);
            // printf("addr %08X \r\n", (uint32_t)(0x08020000 + offset));
        // }
        flash_timer = sys_time_ms();
    }
    return 0;
}

static struct usbd_interface intf0;

void msc_ram_init(uint8_t busid, uintptr_t reg_base)
{
    usbd_desc_register(busid, msc_ram_descriptor);
    usbd_add_interface(busid, usbd_msc_init_intf(busid, &intf0, MSC_OUT_EP, MSC_IN_EP));

    usbd_initialize(busid, reg_base, usbd_event_handler);
}

#if defined(CONFIG_USBDEV_MSC_POLLING)
void msc_ram_polling(uint8_t busid)
{
    usbd_msc_polling(busid);
}
#endif