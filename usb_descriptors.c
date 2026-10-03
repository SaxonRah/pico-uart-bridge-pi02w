// SPDX-License-Identifier: MIT
/*
 * microDOS v29 generic HID UART bridge descriptors.
 */

#include <string.h>
#include "tusb.h"

#define USB_VID 0xCAFE
#define USB_PID 0x4028

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x2900,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&desc_device;
}

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE)
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return desc_hid_report;
}

enum {
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)
#define EPNUM_HID 0x01

uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(
        1,
        ITF_NUM_TOTAL,
        0,
        CONFIG_TOTAL_LEN,
        TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
        100
    ),

    TUD_HID_INOUT_DESCRIPTOR(
        ITF_NUM_HID,
        0,
        HID_ITF_PROTOCOL_NONE,
        sizeof(desc_hid_report),
        EPNUM_HID,
        0x80 | EPNUM_HID,
        CFG_TUD_HID_EP_BUFSIZE,
        1
    )
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL
};

static char const *string_desc_arr[] = {
    (const char[]){0x09, 0x04},
    "microDOS",
    "microDOS HID UART bridge v29",
    "MDHID29"
};

static uint16_t desc_str[64];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    size_t count;

    (void)langid;

    if (index == STRID_LANGID) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        count = 1;
    } else {
        const char *s;

        if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
            return NULL;

        s = string_desc_arr[index];
        count = strlen(s);

        if (count > 63u)
            count = 63u;

        for (size_t i = 0; i < count; ++i)
            desc_str[1 + i] = (uint8_t)s[i];
    }

    desc_str[0] =
        (uint16_t)((TUSB_DESC_STRING << 8) | (2u * count + 2u));

    return desc_str;
}
