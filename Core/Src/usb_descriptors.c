#include "tusb.h"
#include "tusb/tusb_config.h"

static const tusb_desc_device_t DEV_DESC = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,

    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,

    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*) &DEV_DESC;
}

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

enum {
    EPNUM_CDC_NOTIF = 0x81,
    EPNUM_CDC_OUT   = 0x02,
    EPNUM_CDC_IN    = 0x82,
};

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

uint8_t const CONFIG_DESC[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return CONFIG_DESC;
}

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
};

char const* STRING_DESC[] = {
    (const char[]){0x09, 0x04},
    "LRI Electronics",
    "RANDE",
    "000000001",
    "RANDE Serial Interface",
};

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    static uint16_t chars[64] = {0};
    (void) langid;
    size_t chr_count;

    switch(index) {
    case STRID_LANGID:
        memcpy(&chars[1], STRING_DESC[0], 2);
        chr_count = 1;
        break;

    default:
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if(index >= sizeof(STRING_DESC) / sizeof(STRING_DESC[0])) return NULL;

        const char* str = STRING_DESC[index];

        // Cap at max char
        chr_count = strlen(str);
        size_t const max_count = sizeof(chars) / sizeof(chars[0]) - 1; // -1 for string type
        if(chr_count > max_count) chr_count = max_count;

    // Convert ASCII string into UTF-16
        for(size_t i = 0; i < chr_count; i++) {
            chars[1 + i] = str[i];
        }
        break;
    }

    // first byte is length (including header), second byte is string type
    chars[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return chars;
}
