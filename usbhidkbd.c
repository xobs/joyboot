/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "usblink.h"

static const uint8_t class_descriptor[] = {
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

static const struct usb_device_descriptor device_descriptor = {
  .bLength = sizeof(struct usb_device_descriptor),
  .bDescriptorType = DT_DEVICE,       /* DEVICE */
  .bcdUSB = 0x0200,           /* USB 2.0 */
  .bDeviceClass = 0x00,
  .bDeviceSubClass = 0x00,
  .bDeviceProtocol = 0x00,
  .bMaxPacketSize0 = 0x08,    /* 8-byte packets max */
  .idVendor = 0x1bcf,
  .idProduct = 0x05ce,
  .bcdDevice = 0xa014,        /* Device release 1.0 */
  .iManufacturer = 0x00,      /* No manufacturer string */
  .iProduct = 0x00,           /* Product name in string #2 */
  .iSerialNumber = 0x00,      /* No serial number */
  .bNumConfigurations = 0x01,
};

static const struct usb_configuration_descriptor configuration_descriptor = {
  .bLength = sizeof(struct usb_configuration_descriptor),
  .bDescriptorType = DT_CONFIGURATION,
  .wTotalLength = sizeof(struct usb_configuration_descriptor)
                + sizeof(struct usb_interface_descriptor)
                + sizeof(struct usb_hid_descriptor)
                + sizeof(struct usb_endpoint_descriptor),
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0xa0,       /* Remote wakeup supported */
  .bMaxPower = 100/2,         /* 100 mA (in 2-mA units) */
  .data = {
    /* struct usb_interface_descriptor { */
    /*  uint8_t bLength;            */ sizeof(struct usb_interface_descriptor),
    /*  uint8_t bDescriptorType;    */ DT_INTERFACE,
    /*  uint8_t bInterfaceNumber;   */ 0,
    /*  uint8_t bAlternateSetting;  */ 0,
    /*  uint8_t bNumEndpoints;      */ 1, /* One additional EP */
    /*  uint8_t bInterfaceClass;    */ 3, /* HID class */
    /*  uint8_t bInterfaceSubclass; */ 1, /* Boot Device subclass */
    /*  uint8_t bInterfaceProtocol; */ 1, /* 1 == keyboard, 2 == mouse */
    /*  uint8_t iInterface;         */ 0,
    /* }*/

    /* struct usb_hid_descriptor {        */
    /*  uint8_t  bLength;                 */ sizeof(struct usb_hid_descriptor),
    /*  uint8_t  bDescriptorType;         */ DT_HID,
    /*  uint16_t bcdHID;                  */ 0x00, 0x01,
    /*  uint8_t  bCountryCode;            */ 0,
    /*  uint8_t  bNumDescriptors;         */ 1, /* We have only one REPORT */
    /*  uint8_t  bReportDescriptorType;   */ DT_HID_REPORT,
    /*  uint16_t wReportDescriptorLength; */ sizeof(class_descriptor),
                                             sizeof(class_descriptor) >> 8,
    /* }                                  */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ sizeof(struct usb_endpoint_descriptor),
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x81,  /* EP1 (IN) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ 10, /* Every 10 ms */
    /* }                              */
  },

  /*
         0x09 DT_CONFIGURATION 0x3B 0x00 0x02 0x01 0x00 0xA0 0x17

         0x09 DT_INTERFACE 0x00 0x00 0x01 0x03 0x01 0x01 0x00
         0x09 DT_HID 0x00 0x01 0x00 0x01 0x22 0x41 0x00
         0x07 DT_ENDPOINT 0x81 0x03 0x08 0x00 0x0A

         0x09 DT_INTERFACE 0x01 0x00 0x01 0x03 0x01 0x02 0x00
         0x09 DT_HID 0x00 0x01 0x00 0x01 0x22 0xB3 0x00
         0x07 DT_ENDPOINT 0x82 0x03 0x08 0x00 0x0A
  */
};

static int get_string_descriptor(struct USBLink *link,
                                 uint32_t num,
                                 const void **data) {

  static const uint8_t en_us[] = {0x04, DT_STRING, 0x09, 0x04};
  static const uint8_t str[] = {0x06, DT_STRING, 0x65, 0x00, 0x66, 0x00};

  (void)link;

  switch (num) {
  case 0:
    *data = en_us;
    return sizeof(en_us);

  case 1:
    *data = str;
    return sizeof(str);
  }

  return 0;
}

static int get_device_descriptor(struct USBLink *link,
                                 uint32_t num,
                                 const void **data) {

  (void)link;

  if (num == 0) {
    *data = &device_descriptor;
    return sizeof(device_descriptor);
  }
  return 0;
}

static int get_class_descriptor(struct USBLink *link,
                                uint32_t num,
                                const void **data) {

  (void)link;

  if (num == 0) {
    *data = &class_descriptor;
    return sizeof(class_descriptor);
  }

  return 0;
}

static int get_configuration_descriptor(struct USBLink *link,
                                        uint32_t num,
                                        const void **data) {

  (void)link;

  if (num == 0) {
    *data = &configuration_descriptor;
    return configuration_descriptor.wTotalLength;
  }
  return 0;
}

struct USBLink usbHidKbd = {
  .getStringDescriptor = get_string_descriptor,
  .getDeviceDescriptor = get_device_descriptor,
  .getConfigurationDescriptor = get_configuration_descriptor,
  .getClassDescriptor = get_class_descriptor,
};
