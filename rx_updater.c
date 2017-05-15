#include <string.h>

#include "grainuum.h"
#include "kl17.h"
#include "flash.h"

#include "palawan_bl.h"
#include "murmur3.h"

#define BUFFER_SIZE 8
#define NUM_BUFFERS 4
#define EP_INTERVAL_MS 10

/* The area where the bootloader resides */
#define FLASH_PROTECTED_AREA_OFFSET 0
#define FLASH_PROTECTED_AREA_SIZE 8192

static struct GrainuumUSB defaultUsbPhy = {
  /* PTB0 */
  .usbdnIAddr = (uint32_t)&FGPIOB->PDIR,
  .usbdnSAddr = (uint32_t)&FGPIOB->PSOR,
  .usbdnCAddr = (uint32_t)&FGPIOB->PCOR,
  .usbdnDAddr = (uint32_t)&FGPIOB->PDDR,
  .usbdnMask  = (1 << 5),
  .usbdnShift = 5,

  /* PTA4 */
  .usbdpIAddr = (uint32_t)&FGPIOB->PDIR,
  .usbdpSAddr = (uint32_t)&FGPIOB->PSOR,
  .usbdpCAddr = (uint32_t)&FGPIOB->PCOR,
  .usbdpDAddr = (uint32_t)&FGPIOB->PDDR,
  .usbdpMask  = (1 << 6),
  .usbdpShift = 6,
};

void set_usb_config_num(struct GrainuumUSB *usb, int configNum)
{
  (void)usb;
  (void)configNum;
  ;
}

static const uint8_t hid_report_descriptor[] = {
  #if 0
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
  0xc0,        /* END_COLLECTION                         */
#endif
  0x06, 0x00, 0xFF,   // USAGE_PAGE = 0xFF00 (Vendor Defined Page 1)
  0x09, 0x01,         // USAGE (Vendor Usage 1)
  0xA1, 0x01,         // COLLECTION (Application)
    0x15, 0x00,       //      USAGE_MINIMUM (data bytes in the report may have minimum value = 0x00)
    0x26, 0xFF, 0x00, //      USAGE_MAXIMUM (data bytes in the report may have maximum value = 0x00FF = unsigned 255)
    
    0x75, 0x08,       //      REPORT_SIZE (8-bit field size)
    
    0x95, BUFFER_SIZE,//      REPORT_COUNT Make eight 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item)
    0x09, 0x01,       //      USAGE (Undefined)
    0x81, 0x02,       //      INPUT (Data, Array, Abs): Instantiates input packet fields based on the above report size, count, logical min/max, and usage.
    
    0x95, BUFFER_SIZE,//      REPORT_COUNT (Make eight 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item))
    0x09, 0x01,       //      USAGE (Undefined)
    0x91, 0x00,       //      OUTPUT (Data, Array, Abs): Instantiates
                      //      output packet fields.  Uses same report size
                      //      and count as "Input" fields, since nothing
                      //      new/different was specified to the parser
                      //      since the "Input" item.
    
    0x95, BUFFER_SIZE,//      REPORT_COUNT (Make eight 8-bit fields (the next time the parser hits an "Input", "Output", or "Feature" item))
    0x09, 0x01,       //      USAGE (Undefined)
    0xB1, 0x02,       //      Feature: Data
  0xC0 // END_COLLECTION
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
  .wTotalLength = (9  +  /*9 + 9 + 7  +*/  9 + 9 + 7 + 7)/*
                  (sizeof(struct usb_configuration_descriptor)
                + sizeof(struct usb_interface_descriptor)
                + sizeof(struct usb_hid_descriptor)
                + sizeof(struct usb_endpoint_descriptor)*/,
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 5,
  .bmAttributes = 0x80,       /* Remote wakeup not supported */
  .bMaxPower = 100/2,         /* 100 mA (in 2-mA units) */
  .data = {
#if 0
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
    /*  uint16_t wReportDescriptorLength; */ sizeof(class_descriptor),
                                             sizeof(class_descriptor) >> 8,
    /* }                                  */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ 7,
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x81,  /* EP1 (IN) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ EP_INTERVAL_MS, /* Every 6 ms */
    /* }                              */
#endif
    /* struct usb_interface_descriptor { */
    /*  uint8_t bLength;            */ 9,
    /*  uint8_t bDescriptorType;    */ DT_INTERFACE,
    /*  uint8_t bInterfaceNumber;   */ 0,
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
    /*  uint16_t wReportDescriptorLength; */ sizeof(hid_report_descriptor),
                                             sizeof(hid_report_descriptor) >> 8,
    /* }                                  */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ 7,
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x82,  /* EP1 (IN) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ EP_INTERVAL_MS, /* Every 6 ms */
    /* }                              */

    /* struct usb_endpoint_descriptor { */
    /*  uint8_t  bLength;             */ 7,
    /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
    /*  uint8_t  bEndpointAddress;    */ 0x02,  /* EP1 (OUT) */
    /*  uint8_t  bmAttributes;        */ 3,     /* Interrupt */
    /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
    /*  uint8_t  bInterval;           */ EP_INTERVAL_MS, /* Every 6 ms */
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

static int get_string_descriptor(struct GrainuumUSB *usb,
                                 uint32_t num,
                                 const void **data)
{

  static const uint8_t en_us[] = {0x04, DT_STRING, 0x09, 0x04};

  (void)usb;

  if (num == 0) {
    *data = en_us;
    return sizeof(en_us);
  }

  if (num == 1)
    return send_string_descriptor("123", data);

  if (num == 2)
    return send_string_descriptor("21", data);

  if (num == 3)
    return send_string_descriptor("1236", data);

  if (num == 4)
    return send_string_descriptor("12345", data);

  if (num == 5)
    return send_string_descriptor("54", data);

  if (num == 6)
    return send_string_descriptor("12345678901234", data);

  return 0;
}

static int get_device_descriptor(struct GrainuumUSB *usb,
                                 uint32_t num,
                                 const void **data)
{

  (void)usb;

  if (num == 0) {
    *data = &device_descriptor;
    return sizeof(device_descriptor);
  }
  return 0;
}

static int get_hid_report_descriptor(struct GrainuumUSB *usb,
                                     uint32_t num,
                                     const void **data)
{

  (void)usb;

  if (num == 0) {
    *data = &hid_report_descriptor;
    return sizeof(hid_report_descriptor);
  }

  return 0;
}

static int get_configuration_descriptor(struct GrainuumUSB *usb,
                                        uint32_t num,
                                        const void **data)
{

  (void)usb;

  if (num == 0) {
    *data = &configuration_descriptor;
    return configuration_descriptor.wTotalLength;
  }
  return 0;
}

static int get_descriptor(struct GrainuumUSB *usb,
                          const void *packet,
                          const void **response)
{

  const struct usb_setup_packet *setup = packet;

  switch (setup->wValueH) {
  case DT_DEVICE:
    return get_device_descriptor(usb, setup->wValueL, response);

  case DT_STRING:
    return get_string_descriptor(usb, setup->wValueL, response);

  case DT_CONFIGURATION:
    return get_configuration_descriptor(usb, setup->wValueL, response);

  case DT_HID_REPORT:
    return get_hid_report_descriptor(usb, setup->wValueL, response);
  }

  return 0;
}

uint32_t rx_buffer[NUM_BUFFERS][BUFFER_SIZE / sizeof(uint32_t)];
static uint8_t rx_buffer_head;
static uint8_t rx_buffer_tail;

uint32_t rx_buffer_queries = 0;
static void * get_usb_rx_buffer(struct GrainuumUSB *usb,
                                uint8_t epNum,
                                int32_t *size)
{
  (void)usb;
  (void)epNum;

  if (size)
    *size = sizeof(rx_buffer[0]);
  rx_buffer_queries++;
  return rx_buffer[rx_buffer_head];
}

static int received_data(struct GrainuumUSB *usb,
                         uint8_t epNum,
                         uint32_t bytes,
                         const void *data)
{
  (void)usb;
  (void)epNum;
  (void)bytes;
  (void)data;

  if (epNum == 2) {
    rx_buffer_head = (rx_buffer_head + 1) & (NUM_BUFFERS - 1);
    //asm("bkpt #3");
  }

  /* Return 0, indicating this packet is complete. */
  return 0;
}

static int send_data_finished(struct GrainuumUSB *usb, int result)
{
  (void)usb;
  (void)result;

  return 0;
}

static struct GrainuumConfig hid_link = {
  .getDescriptor              = get_descriptor,
  .getReceiveBuffer           = get_usb_rx_buffer,
  .receiveData                = received_data,
  .sendDataFinished           = send_data_finished,
  .setConfigNum               = set_usb_config_num,
};

static GRAINUUM_BUFFER(phy_queue, 4);

void VectorBC(void)
{
  grainuumCaptureI(&defaultUsbPhy, GRAINUUM_BUFFER_ENTRY(phy_queue));

  /* Clear all pending interrupts on this port. */
  PORTB->ISFR = 0xFFFFFFFF;
}

void grainuumReceivePacket(struct GrainuumUSB *usb)
{
  (void)usb;
  GRAINUUM_BUFFER_ADVANCE(phy_queue);
}

void grainuumInitPre(struct GrainuumUSB *usb)
{
  (void)usb;
  GRAINUUM_BUFFER_INIT(phy_queue);
}

static void process_next_usb_event(struct GrainuumUSB *usb) {
  if (!GRAINUUM_BUFFER_IS_EMPTY(phy_queue)) {
    uint8_t *in_ptr = (uint8_t *)GRAINUUM_BUFFER_TOP(phy_queue);

    // Advance to the next packet (allowing us to be reentrant)
    GRAINUUM_BUFFER_REMOVE(phy_queue);

    // Process the current packet
    grainuumProcess(usb, in_ptr);

    return;
  }
}

int done;
int num_sent = 0;
int num_failed = 0;

static int do_erase_flash(struct bl_state *state, struct bl_pkt *result,
                          uint32_t address, uint32_t count)
{
  int ret;

  // Make sure we're not overwriting the bootloader
  if ((address < (FLASH_PROTECTED_AREA_OFFSET + FLASH_PROTECTED_AREA_SIZE)) && (state->flash_is_protected))
    return address_out_of_range;

  // XXX If the protected area isn't located at offset 0, then this could extend into it

  // Make sure we don't run off the end of flash
  if ((address + count) > P_FLASH_SIZE)
    return address_out_of_range;

  ret = flashEraseSectors(address / FTFx_PSECTOR_SIZE, count / FTFx_PSECTOR_SIZE);

  if (ret != F_ERR_OK) {
    result->result.extra = ret;
    return subsystem_error;
  }

  return no_error;
}

static int do_program_flash(struct bl_state *state, struct bl_pkt *result,
                            uint32_t value)
{
  int ret;
  uint32_t offset;

  offset = state->offset;

  if ((offset >= FLASH_PROTECTED_AREA_OFFSET) && (offset < (FLASH_PROTECTED_AREA_OFFSET + FLASH_PROTECTED_AREA_SIZE)))
    return address_out_of_range;

  ret = flashProgram((uint8_t *)&value, (uint8_t *)&offset, sizeof(value));
  if (ret != F_ERR_OK) {
    result->result.extra = ret;
    return subsystem_error;
  }

  state->offset += 4;
  return no_error;
}

static int do_set_address(struct bl_state *state, struct bl_pkt *result,
                          uint32_t offset)
{
  (void)result;

  if (offset > P_FLASH_SIZE)
    return address_out_of_range;

  if ((offset > FLASH_PROTECTED_AREA_OFFSET) && (offset < (FLASH_PROTECTED_AREA_OFFSET + FLASH_PROTECTED_AREA_SIZE)))
    return address_out_of_range;

  state->offset = offset;
  return no_error;
}

static int do_echo_back(struct bl_state *state, struct bl_pkt *result,
                        const struct bl_pkt *packet)
{
  (void)state;
  memcpy(result, packet, sizeof(*result));
  return no_error;
}

static int do_hash_memory(struct bl_state *state, struct bl_pkt *result,
                          const struct bl_pkt *packet)
{
  (void)state;
  uint32_t start = packet->hash.offset;
  uint32_t length = packet->hash.size;

  if (start & 3)
    return address_not_valid;
  
  if (length & 3)
    return size_not_valid;

  MurmurHash3_x86_32((const void *)start, length, 0, result->result.reserved);

  return no_error;
}

static int process_one_packet(struct bl_state *state,
                              struct bl_pkt *result,
                              const struct bl_pkt *packet)
{

  switch (packet->cmd) {

  case erase_block:
    return do_erase_flash(state, result, packet->erase_sector.offset, packet->erase_sector.count);

  case program_value:
    return do_program_flash(state, result, packet->program.data);

  case set_address:
    return do_set_address(state, result, packet->set_address.offset);

  case echo_back:
    return do_echo_back(state, result, packet);

  case hash_memory:
    return do_hash_memory(state, result, packet);
  }

  return unhandled_command;
}

int updateRx(void)
{
  struct bl_state state = {0};
  int last_packet_num = -1;
  static struct bl_pkt result_pkt = {0};

  state.flash_is_protected = 1;

  /* Unlock PORTA and PORTB */
  SIM->SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTB;

  /* Set up D+ and D- as slow-slew GPIOs (pin mux type 1), and enable IRQs */
  PORTB->PCR[5] = (1 << 8) | (0xb << 16) | (1 << 2);
  PORTB->PCR[6] = (1 << 8) | (1 << 2);

  grainuumInit(&defaultUsbPhy, &hid_link);

  {
    int i;
    for (i = 0; i < 1000; i++) {
      int j;
      for (j = 0; j < 77; j++) {
        asm("");
      }
    }
  }

  /* Enable PORTB IRQ */
  NVIC_EnableIRQ(PINB_IRQn);
  __enable_irq();

  grainuumConnect(&defaultUsbPhy);

  while (!done) {
    process_next_usb_event(&defaultUsbPhy);

    // If the rx_buffer_head has advnaced, then we have data to process in EP2
    if (rx_buffer_head != rx_buffer_tail) {
      struct bl_pkt *incoming_pkt = (struct bl_pkt *)rx_buffer[rx_buffer_tail];

      // If the last packet num is the same as this packet num, then we've processed
      // this packet already, but were unsuccessful in sending the packet.  If not,
      // as is the case here, handle it as a new packet.
      if (rx_buffer_tail != last_packet_num) {
        last_packet_num = rx_buffer_tail;
        result_pkt.seq_num = incoming_pkt->seq_num;
        result_pkt.cmd = packet_result;

        unsigned int i;
        for (i = 0; i < sizeof(result_pkt.raw.data); i++)
          result_pkt.raw.data[i] = 0;

        result_pkt.result.code = process_one_packet(&state, &result_pkt, incoming_pkt);
      }

      // Advance the packet number only if this packet was sent successfully
      if (!grainuumSendData(&defaultUsbPhy, 2, &result_pkt, sizeof(result_pkt))) {
        rx_buffer_tail = (rx_buffer_tail + 1) & (NUM_BUFFERS - 1);
      }
    }
  }

  return 0;
}
