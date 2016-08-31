#include <string.h>

#include "usbphy.h"
#include "usbmac.h"
#include "usblink.h"
#include "kl17.h"

#define BUFFER_SIZE 8

static struct USBPHY defaultUsbPhy = {
  /* PTB0 */
  .usbdnIAddr = (uint32_t)&FGPIOB->PDIR,
  .usbdnSAddr = (uint32_t)&FGPIOB->PSOR,
  .usbdnCAddr = (uint32_t)&FGPIOB->PCOR,
  .usbdnDAddr = (uint32_t)&FGPIOB->PDDR,
  .usbdnMask  = (1 << 0),
  .usbdnShift = 0,

  /* PTA4 */
  .usbdpIAddr = (uint32_t)&FGPIOA->PDIR,
  .usbdpSAddr = (uint32_t)&FGPIOA->PSOR,
  .usbdpCAddr = (uint32_t)&FGPIOA->PCOR,
  .usbdpDAddr = (uint32_t)&FGPIOA->PDDR,
  .usbdpMask  = (1 << 4),
  .usbdpShift = 4,
};

void set_usb_config_num(struct USBLink *link, int configNum)
{
  (void)link;
  (void)configNum;
  ;
}

static const uint8_t kbd_class_descriptor[] = {
  0x05, 0x01, /* USAGE_PAGE (Generic Desktop)           */
  0x09, 0x06, /* USAGE (Keyboard)                       */
  0xa1, 0x01, /* COLLECTION (Application)               */
  0x05, 0x07, /*   USAGE_PAGE (Keyboard)                */
  0x19, 0xe0, /*   USAGE_MINIMUM (Keyboard LeftControl) */
  0x29, 0xe7, /*   USAGE_MAXIMUM (Keyboard Right GUI)   */
  0x15, 0x00, /*   LOGICAL_MINIMUM (0)                  */
  0x25, 0x01, /*   LOGICAL_MAXIMUM (1)                  */
  0x75, 0x01, /*   REPORT_SIZE (1)                      */
  0x95, 0x08, /*   REPORT_COUNT (8)                     */
  0x81, 0x02, /*   INPUT (Data,Var,Abs)                 */
  0x95, 0x01, /*   REPORT_COUNT (1)                     */
  0x75, 0x08, /*   REPORT_SIZE (8)                      */
  0x81, 0x03, /*   INPUT (Cnst,Var,Abs)                 */
  0x95, 0x05, /*   REPORT_COUNT (5)                     */
  0x75, 0x01, /*   REPORT_SIZE (1)                      */
  0x05, 0x08, /*   USAGE_PAGE (LEDs)                    */
  0x19, 0x01, /*   USAGE_MINIMUM (Num Lock)             */
  0x29, 0x05, /*   USAGE_MAXIMUM (Kana)                 */
  0x91, 0x02, /*   OUTPUT (Data,Var,Abs)                */
  0x95, 0x01, /*   REPORT_COUNT (1)                     */
  0x75, 0x03, /*   REPORT_SIZE (3)                      */
  0x91, 0x03, /*   OUTPUT (Cnst,Var,Abs)                */
  0x95, 0x06, /*   REPORT_COUNT (6)                     */
  0x75, 0x08, /*   REPORT_SIZE (8)                      */
  0x15, 0x00, /*   LOGICAL_MINIMUM (0)                  */
  0x25, 0x65, /*   LOGICAL_MAXIMUM (101)                */
  0x05, 0x07, /*   USAGE_PAGE (Keyboard)                */
  0x19, 0x00, /*   USAGE_MINIMUM (Reserved)             */
  0x29, 0x65, /*   USAGE_MAXIMUM (Keyboard Application) */
  0x81, 0x00, /*   INPUT (Data,Ary,Abs)                 */
  0xc0        /* END_COLLECTION                         */
};

static const uint8_t class_descriptor[] = {
  0x06, 0x00, 0xFF,   // Usage Page = 0xFF00 (Vendor Defined Page 1)
  0x09, 0x01,         // Usage (Vendor Usage 1)
  0xA1, 0x01,         // Collection (Application)
    0x15, 0x00,       //      Logical Minimum (data bytes in the report may have minimum value = 0x00)
    0x26, 0xFF, 0x00, //      Logical Maximum (data bytes in the report may have maximum value = 0x00FF = unsigned 255)
    0x75, 0x08,       //      Report Size: 8-bit field size
    0x95, BUFFER_SIZE,//      Report Count: Make eight 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item)
    0x09, 0x01,       //      Usage: Undefined
    0x81, 0x02,       //      Input (Data, Array, Abs): Instantiates input packet fields based on the above report size, count, logical min/max, and usage.
    0x95, BUFFER_SIZE,//      Report Count: Make eight 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item)
    0x09, 0x01,       //      Usage: Undefined
    0x91, 0x00,       //      Output (Data, Array, Abs): Instantiates
                      //      output packet fields.  Uses same report size
                      //      and count as "Input" fields, since nothing
                      //      new/different was specified to the parser
                      //      since the "Input" item.
    0x95, BUFFER_SIZE,//      Report Count: Make eight 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item)
    0x09, 0x01,       //      Usage: Undefined
    0xB1, 0x02,       //      Feature: Data
  0xC0 // End Collection
};

static const struct usb_device_descriptor device_descriptor = {
  .bLength = 18,//sizeof(struct usb_device_descriptor),
  .bDescriptorType = DT_DEVICE,       /* DEVICE */
  .bcdUSB = 0x0111,           /* USB 1.11 */
  .bDeviceClass = 0x00,
  .bDeviceSubClass = 0x00,
  .bDeviceProtocol = 0x00,
  .bMaxPacketSize0 = 0x08,    /* 8-byte packets max */
  .idVendor = 0x1bcf,
  .idProduct = 0x05ce,
  .bcdDevice = 0xa014,        /* Device release 1.0 */
  .iManufacturer = 0x02,      /* No manufacturer string */
  .iProduct = 0x01,           /* Product name in string #2 */
  .iSerialNumber = 0x03,      /* No serial number */
  .bNumConfigurations = 0x01,
};

static const struct usb_configuration_descriptor configuration_descriptor = {
  .bLength = 9,//sizeof(struct usb_configuration_descriptor),
  .bDescriptorType = DT_CONFIGURATION,
  .wTotalLength = (9  +  9 + 9 + 7  +  9 + 9 + 7 + 7)/*
                  (sizeof(struct usb_configuration_descriptor)
                + sizeof(struct usb_interface_descriptor)
                + sizeof(struct usb_hid_descriptor)
                + sizeof(struct usb_endpoint_descriptor)*/,
  .bNumInterfaces = 2,
  .bConfigurationValue = 1,
  .iConfiguration = 5,
  .bmAttributes = 0xa0,       /* Remote wakeup supported */
  .bMaxPower = 100/2,         /* 100 mA (in 2-mA units) */
  .data = {
    /* struct usb_interface_descriptor { */
    /*  uint8_t bLength;            */ 9,
    /*  uint8_t bDescriptorType;    */ DT_INTERFACE,
    /*  uint8_t bInterfaceNumber;   */ 0,
    /*  uint8_t bAlternateSetting;  */ 0,
    /*  uint8_t bNumEndpoints;      */ 1, /* One extra EPs */
    /*  uint8_t bInterfaceClass;    */ 3, /* HID class */
    /*  uint8_t bInterfaceSubclass; */ 1, /* Boot Device subclass */
    /*  uint8_t bInterfaceProtocol; */ 1, /* 1 == keyboard, 2 == mouse */
    /*  uint8_t iInterface;         */ 6, /* String index #6 */
    /* }*/

    /* struct usb_hid_descriptor {        */
    /*  uint8_t  bLength;                 */ 9,
    /*  uint8_t  bDescriptorType;         */ DT_HID,
    /*  uint16_t bcdHID;                  */ 0x11, 0x01,
    /*  uint8_t  bCountryCode;            */ 0,
    /*  uint8_t  bNumDescriptors;         */ 1, /* We have only one REPORT */
    /*  uint8_t  bReportDescriptorType;   */ DT_HID_REPORT,
    /*  uint16_t wReportDescriptorLength; */ sizeof(kbd_class_descriptor),
                                             sizeof(kbd_class_descriptor) >> 8,
    /* }                                  */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ 7,
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x81,  /* EP1 (IN) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ 6, /* Every 1 ms */
    /* }                              */

    /* struct usb_interface_descriptor { */
    /*  uint8_t bLength;            */ 9,
    /*  uint8_t bDescriptorType;    */ DT_INTERFACE,
    /*  uint8_t bInterfaceNumber;   */ 1,
    /*  uint8_t bAlternateSetting;  */ 0,
    /*  uint8_t bNumEndpoints;      */ 2, /* Two extra EPs */
    /*  uint8_t bInterfaceClass;    */ 3, /* HID class */
    /*  uint8_t bInterfaceSubclass; */ 0, /* Boot Device subclass */
    /*  uint8_t bInterfaceProtocol; */ 0, /* 1 == keyboard, 2 == mouse */
    /*  uint8_t iInterface;         */ 4, /* String index #4 */
    /* }*/

    /* struct usb_hid_descriptor {        */
    /*  uint8_t  bLength;                 */ 9,
    /*  uint8_t  bDescriptorType;         */ DT_HID,
    /*  uint16_t bcdHID;                  */ 0x11, 0x01,
    /*  uint8_t  bCountryCode;            */ 0,
    /*  uint8_t  bNumDescriptors;         */ 1, /* We have only one REPORT */
    /*  uint8_t  bReportDescriptorType;   */ DT_HID_REPORT,
    /*  uint16_t wReportDescriptorLength; */ sizeof(class_descriptor),
                                             sizeof(class_descriptor) >> 8,
    /* }                                  */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ 7,
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x82,  /* EP1 (IN) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ 6, /* Every 1 ms */
    /* }                              */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ 7,
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x02,  /* EP1 (OUT) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ 6, /* Every 1 ms */
    /* }                              */
  },
};

#define USB_STR_BUF_LEN 64

uint32_t str_buf_storage[USB_STR_BUF_LEN / sizeof(uint32_t)];
static int send_string_descriptor(const char *str, const void **data)
{
  int len;
  int max_len;
  uint8_t *str_buf = (uint8_t *)str_buf_storage;
  uint8_t *str_offset = str_buf;

  len = strlen(str);
  max_len = (USB_STR_BUF_LEN / 2) - 2;

  if (len > max_len)
    len = max_len;

  *str_offset++ = (len * 2) + 2;  // Two bytes for length count
  *str_offset++ = DT_STRING;      // Sending a string descriptor

  while (len--) {
    *str_offset++ = *str++;
    *str_offset++ = 0;
  }

  *data = str_buf;

  // Return the size, which is stored in the first byte of the output data.
  return str_buf[0];
}

static int get_string_descriptor(struct USBLink *link,
                                 uint32_t num,
                                 const void **data)
{

  static const uint8_t en_us[] = {0x04, DT_STRING, 0x09, 0x04};

  (void)link;

  if (num == 0) {
    *data = en_us;
    return sizeof(en_us);
  }

  if (num == 1)
    return send_string_descriptor("10", data);

  if (num == 2)
    return send_string_descriptor("21", data);

  if (num == 3)
    return send_string_descriptor("32", data);

  if (num == 4)
    return send_string_descriptor("43", data);

  if (num == 5)
    return send_string_descriptor("54", data);

  if (num == 6)
    return send_string_descriptor("65", data);

  return 0;
}

static int get_device_descriptor(struct USBLink *link,
                                 uint32_t num,
                                 const void **data)
{

  (void)link;

  if (num == 0) {
    *data = &device_descriptor;
    return sizeof(device_descriptor);
  }
  return 0;
}

static int get_class_descriptor(struct USBLink *link,
                                uint32_t num,
                                const void **data)
{

  (void)link;

  if (num == 0) {
    *data = &class_descriptor;
    return sizeof(class_descriptor);
  }

  return 0;
}

static int get_configuration_descriptor(struct USBLink *link,
                                        uint32_t num,
                                        const void **data)
{

  (void)link;

  if (num == 0) {
    *data = &configuration_descriptor;
    return configuration_descriptor.wTotalLength;
  }
  return 0;
}

static int get_default_descriptor(struct USBLink *link,
                                  const struct usb_mac_setup_packet *setup,
                                  const void **data)
{
  (void)link;
  (void)setup;
  (void)data;

  /*
  if (setup->bmRequestType == 0xa1) {
    static uint8_t dumb_features[] = {0x01, 0x23, 0x45};
    *data = dumb_features;
    return sizeof(dumb_features);
  }
  */

  return 0;
}

uint32_t rx_buffer[BUFFER_SIZE / sizeof(uint32_t)];
static void * get_usb_rx_buffer(struct USBLink *link, uint8_t epNum, int32_t *size)
{
  (void)link;
  (void)epNum;

  if (size)
    *size = sizeof(rx_buffer);
  return rx_buffer;
}

static void *ep2_buffer;
static uint32_t ep2_buffer_size;

static void * get_usb_tx_buffer(struct USBLink *link, uint8_t epNum, int32_t *size)
{
  (void)link;
  (void)epNum;

  if (epNum == 2) {
    if (size)
      *size = ep2_buffer_size;
    return ep2_buffer;
  }

  return NULL;
}

static int received_data(struct USBLink *link,
                         uint8_t epNum,
                         uint32_t bytes,
                         const void *data)
{
  (void)link;
  (void)epNum;
  (void)bytes;
  (void)data;

  if (epNum == 2) {
    static uint8_t buffer[] = {0xde, 0xad, 0xbe, 0xef, 0xaa, 0x55, 0xff, 0x00};

    ep2_buffer_size = sizeof(buffer);
    ep2_buffer = buffer;
  }

  /* Return 0, indicating this packet is complete. */
  return 0;
}

static int send_data_finished(struct USBLink *link, uint8_t epNum, const void *data)
{
  (void)link;
  (void)epNum;
  (void)data;

  return 0;
}

static struct USBLink hid_link = {
  .getStringDescriptor        = get_string_descriptor,
  .getDeviceDescriptor        = get_device_descriptor,
  .getConfigurationDescriptor = get_configuration_descriptor,
  .getClassDescriptor         = get_class_descriptor,
  .getDescriptor              = get_default_descriptor,
  .setConfigNum               = set_usb_config_num,
  .getSendBuffer              = get_usb_tx_buffer,
  .getReceiveBuffer           = get_usb_rx_buffer,
  .sendData                   = send_data_finished,
  .receiveData                = received_data,
};


void VectorB8(void)
{
  usbCaptureI(&defaultUsbPhy);

  /* Clear all pending interrupts on this port. */
  PORTA->ISFR = 0xFFFFFFFF;
}

int done;

int updateRx(void)
{
  /* Unlock PORTA and PORTB */
  SIM->SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTB;

  /* Set up D+ and D- as GPIOs (pin mux type 1), and enable IRQs */
  PORTA->PCR[4] = (1 << 8) | (0xb << 16);
  PORTB->PCR[0] = (1 << 8);

  usbMacInit(usbMacDefault(), &hid_link);
  usbPhyInit(&defaultUsbPhy, usbMacDefault());
  hid_link.mac = usbMacDefault();

  {
    int i;
    for (i = 0; i < 1000; i++) {
      int j;
      for (j = 0; j < 77; j++) {
        asm("");
      }
    }
  }

  /* Enable PORTA IRQ */
  NVIC_EnableIRQ(30);
  __enable_irq();

  usbPhyAttach(&defaultUsbPhy);

  while (!done) {
    usbPhyWorker(&defaultUsbPhy);
  }

  return 0;
}
