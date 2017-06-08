#include <string.h>

#include "grainuum.h"
#include "kl17.h"
#include "flash.h"
#include "memio.h"

#include "palawan_bl.h"
#include "murmur3.h"

#define BUFFER_SIZE 8
#define NUM_BUFFERS 4
#define EP_INTERVAL_MS 6

/* The area where the bootloader resides */
#define FLASH_PROTECTED_AREA_OFFSET 0
#define FLASH_PROTECTED_AREA_SIZE 8192

static struct GrainuumUSB defaultUsbPhy = {
    /* PTB6 */
    .usbdnIAddr = (uint32_t)&FGPIOB->PDIR,
    .usbdnSAddr = (uint32_t)&FGPIOB->PSOR,
    .usbdnCAddr = (uint32_t)&FGPIOB->PCOR,
    .usbdnDAddr = (uint32_t)&FGPIOB->PDDR,
    .usbdnMask = (1 << 6),
    .usbdnShift = 6,

    /* PTB5 */
    .usbdpIAddr = (uint32_t)&FGPIOB->PDIR,
    .usbdpSAddr = (uint32_t)&FGPIOB->PSOR,
    .usbdpCAddr = (uint32_t)&FGPIOB->PCOR,
    .usbdpDAddr = (uint32_t)&FGPIOB->PDDR,
    .usbdpMask = (1 << 5),
    .usbdpShift = 5,
};

static void set_usb_config_num(struct GrainuumUSB *usb, int configNum)
{
  (void)usb;
  (void)configNum;
  ;
}

static const uint8_t hid_report_descriptor[] = {
    0x06, 0x00, 0xFF, // (GLOBAL) USAGE_PAGE         0xFF00 Vendor-defined
    0x09, 0x00,       // (LOCAL)  USAGE              0xFF000000
    0xA1, 0x01,       // (MAIN)   COLLECTION         0x01 Application (Usage=0xFF000000: Page=Vendor-defined, Usage=, Type=)
    0x26, 0xFF, 0x00, //   (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255)
    0x75, 0x08,       //   (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x08,       //   (GLOBAL) REPORT_COUNT       0x08 (8) Number of fields
    0x06, 0xFF, 0xFF, //   (GLOBAL) USAGE_PAGE         0xFFFF Vendor-defined
    0x09, 0x01,       //   (LOCAL)  USAGE              0xFFFF0001
    0x81, 0x02,       //   (MAIN)   INPUT              0x00000002 (8 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0x09, 0x01,       //   (LOCAL)  USAGE              0xFFFF0001
    0x91, 0x02,       //   (MAIN)   OUTPUT             0x00000002 (8 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap
    0xC0,             // (MAIN)   END_COLLECTION     Application
};

static const struct usb_device_descriptor device_descriptor = {
    .bLength = 18,                //sizeof(struct usb_device_descriptor),
    .bDescriptorType = DT_DEVICE, /* DEVICE */
    .bcdUSB = 0x0200,             /* USB 2.0 */
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = 0x08, /* 8-byte packets max */
    .idVendor = 0x1209,
    .idProduct = 0x9317,
    .bcdDevice = 0x0114,   /* Device release 1.14 */
    .iManufacturer = 0x02, /* No manufacturer string */
    .iProduct = 0x01,      /* Product name in string #2 */
    .iSerialNumber = 0x03, /* No serial number */
    .bNumConfigurations = 0x01,
};

static const struct usb_configuration_descriptor configuration_descriptor = {
    .bLength = 9, //sizeof(struct usb_configuration_descriptor),
    .bDescriptorType = DT_CONFIGURATION,
    .wTotalLength = (9 + /*9 + 9 + 7  +*/ 9 + 9 + 7 + 7) /*
                  (sizeof(struct usb_configuration_descriptor)
                + sizeof(struct usb_interface_descriptor)
                + sizeof(struct usb_hid_descriptor)
                + sizeof(struct usb_endpoint_descriptor)*/,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 5,
    .bmAttributes = 0x80, /* Remote wakeup not supported */
    .bMaxPower = 100 / 2, /* 100 mA (in 2-mA units) */
    .data = {
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
        /*  uint8_t  bEndpointAddress;    */ 0x81, /* EP1 (IN) */
        /*  uint8_t  bmAttributes;        */ 3,    /* Interrupt */
        /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
        /*  uint8_t  bInterval;           */ EP_INTERVAL_MS, /* Every 6 ms */
                                                             /* }                              */

        /* struct usb_endpoint_descriptor { */
        /*  uint8_t  bLength;             */ 7,
        /*  uint8_t  bDescriptorType;     */ DT_ENDPOINT,
        /*  uint8_t  bEndpointAddress;    */ 0x01, /* EP1 (OUT) */
        /*  uint8_t  bmAttributes;        */ 3,    /* Interrupt */
        /*  uint16_t wMaxPacketSize;      */ 0x08, 0x00,
        /*  uint8_t  bInterval;           */ EP_INTERVAL_MS, /* Every 6 ms */
                                                             /* }                              */
    },
};

#define USB_STR_BUF_LEN 64

static uint32_t str_buf_storage[USB_STR_BUF_LEN / sizeof(uint32_t)];
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

  *str_offset++ = (len * 2) + 2; // Two bytes for length count
  *str_offset++ = DT_STRING;     // Sending a string descriptor

  while (len--)
  {
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

  if (num == 0)
  {
    *data = en_us;
    return sizeof(en_us);
  }

  // Product
  if (num == 1)
    return send_string_descriptor("Palawan Bootloader", data);

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

  if (num == 0)
  {
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

  if (num == 0)
  {
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

  if (num == 0)
  {
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

  switch (setup->wValueH)
  {
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

static uint32_t rx_buffer[NUM_BUFFERS][BUFFER_SIZE / sizeof(uint32_t)];
static uint8_t rx_buffer_head;
static uint8_t rx_buffer_tail;

static uint32_t rx_buffer_queries = 0;
static void *get_usb_rx_buffer(struct GrainuumUSB *usb,
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

  if (epNum == 1)
  {
    rx_buffer_head = (rx_buffer_head + 1) & (NUM_BUFFERS - 1);
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
    .getDescriptor = get_descriptor,
    .getReceiveBuffer = get_usb_rx_buffer,
    .receiveData = received_data,
    .sendDataFinished = send_data_finished,
    .setConfigNum = set_usb_config_num,
};

static GRAINUUM_BUFFER(phy_queue, 8);

__attribute__((section(".ramtext"))) static void handle_usb_packet(void)
{
  grainuumCaptureI(&defaultUsbPhy, GRAINUUM_BUFFER_ENTRY(phy_queue));

  /* Clear all pending interrupts on this port. */
  PORTB->ISFR = 0xFFFFFFFF;
}

void VectorBC(void)
{
  handle_usb_packet();
}

__attribute__((section(".ramtext"))) void grainuumReceivePacket(struct GrainuumUSB *usb)
{
  (void)usb;
  GRAINUUM_BUFFER_ADVANCE(phy_queue);
}

void grainuumInitPre(struct GrainuumUSB *usb)
{
  (void)usb;
  GRAINUUM_BUFFER_INIT(phy_queue);
}

/*
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
*/

static void process_all_usb_events(struct GrainuumUSB *usb)
{
  while (!GRAINUUM_BUFFER_IS_EMPTY(phy_queue))
  {
    uint8_t *in_ptr = (uint8_t *)GRAINUUM_BUFFER_TOP(phy_queue);

    // Advance to the next packet (allowing us to be reentrant)
    GRAINUUM_BUFFER_REMOVE(phy_queue);

    // Process the current packet
    grainuumProcess(usb, in_ptr);
  }
}

static int done;
static uint32_t erase_flash_address;
static uint32_t erase_flash_count;

__attribute__((section(".ramtext"))) static void read_usb_if_exists(void)
{
  // If an interrupt is pending, process the USB packet.
  if (PORTB->ISFR)
    handle_usb_packet();
}

static int erase_flash_callback(struct bl_state *state, struct result_pkt *result, void *arg)
{
  (void)arg;
  int ret;

  result->small = no_error;
  ret = flashEraseSectors(erase_flash_address++, 1, read_usb_if_exists);
  erase_flash_count--;

  if (ret != F_ERR_OK)
  {
    result->large = ret;
    result->medium = erase_flash_address;
    result->small = subsystem_error;
    state->continue_function = NULL;
    return 0;
  }

  // If there are no more sectors to update, finish up.
  if (erase_flash_count <= 0)
    return 0;
  return 1;
}

static int do_erase_flash(struct bl_state *state, struct result_pkt *result,
                          uint32_t address, uint32_t count)
{
  // Make sure we're not overwriting the bootloader
  extern uint32_t __bl_size__;
  if ((address * FTFx_PSECTOR_SIZE < (uint32_t)&__bl_size__) && (state->flash_is_protected))
    return address_out_of_range;

  // XXX If the protected area isn't located at offset 0, then this could extend into it

  // Make sure we don't run off the end of flash
  if ((address * FTFx_PSECTOR_SIZE + count * FTFx_PSECTOR_SIZE) > P_FLASH_SIZE)
    return address_out_of_range;

  erase_flash_address = address;
  erase_flash_count = count;
  state->continue_function = erase_flash_callback;

  result->large = count;

  return no_error;
}

static uint32_t peek_poke_read(uint8_t type, uint32_t address)
{
  switch (type & PEEK_POKE_SIZE_MASK)
  {
  case PEEK_POKE_SIZE_8:
    return readb(address);
  case PEEK_POKE_SIZE_16:
    return readw(address);
  case PEEK_POKE_SIZE_32:
    return readl(address);
  default:
    return -1;
  }
}

// Number of bytes for the given operation
static uint32_t peek_poke_size(uint8_t type)
{
  switch (type & PEEK_POKE_SIZE_MASK)
  {
  case PEEK_POKE_SIZE_8:
    return 1;
  case PEEK_POKE_SIZE_16:
    return 2;
  default:
    return 4;
  }
}

static int do_peek_poke(struct bl_state *state, struct result_pkt *result,
                        const struct read_write_pkt *peek_poke)
{
  (void)state;
  static uint32_t address; // Persistent address for all operations

  if (peek_poke->type & PEEK_POKE_READ)
  {
    address = peek_poke->address;
    result->large = peek_poke_read(peek_poke->type, address);
  }
  else
  {
    uint32_t value = peek_poke->value;

    // If an operation was specified, perform it.
    switch (peek_poke->type & PEEK_POKE_OP_MASK)
    {
    case PEEK_POKE_OP_NONE:
      break;
    case PEEK_POKE_OP_SET:
      value = peek_poke_read(peek_poke->type, address) | value;
      break;
    case PEEK_POKE_OP_CLR:
      value = peek_poke_read(peek_poke->type, address) & ~value;
      break;
    case PEEK_POKE_OP_TOG:
      value = peek_poke_read(peek_poke->type, address) ^ value;
      break;
    }

    // Write the computed value to memory
    switch (peek_poke->type & PEEK_POKE_SIZE_MASK)
    {
    case PEEK_POKE_SIZE_8:
      writeb(value, address);
      break;
    case PEEK_POKE_SIZE_16:
      writew(value, address);
      break;
    case PEEK_POKE_SIZE_32:
      writel(value, address);
      break;
    }
  }

  // If an increment/decrement was requested, perform that operation
  switch (peek_poke->type & PEEK_POKE_INCR_MASK)
  {
  case PEEK_POKE_INCR:
    address += peek_poke->increment * peek_poke_size(peek_poke->type);
    break;
  case PEEK_POKE_DECR:
    address -= peek_poke->increment * peek_poke_size(peek_poke->type);
    break;
  }

  return no_error;
}

static int do_start_programming(struct bl_state *state, struct result_pkt *result,
                                const struct start_programming_pkt *pkt)
{
  (void)result;
  uint32_t offset = pkt->offset;
  uint32_t size = pkt->count;
  extern uint32_t __app_start__;
  extern uint32_t __app_end__;

  if (offset > (P_FLASH_SIZE * FTFx_PSECTOR_SIZE))
  {
    asm("bkpt #52");
    return address_out_of_range;
  }

  if ((offset < ((uint32_t)&__app_start__)) || ((offset + size) > ((uint32_t)&__app_end__)))
  {
    asm("bkpt #53");
    return address_out_of_range;
  }

  if (offset & 3)
  {
    asm("bkpt #54");
    return address_not_valid;
  }

  if (size & 3)
  {
    asm("bkpt #55");
    return size_not_valid;
  }

  state->offset = offset;
  state->count = size;
  state->buffer_offset = 0;

  return no_error;
}

static int do_program_data(struct bl_state *state, struct result_pkt *result,
                           const struct program_pkt *pkt)
{
  uint32_t byte;
  for (byte = 0; byte < sizeof(pkt->data) && state->count > 0; byte++)
  {
    state->buffer[state->buffer_offset++] = pkt->data[byte];

    if (state->buffer_offset > 3)
    {

      // If the flash isn't erased, throw an error.
      if (readl(state->offset) != 0xffffffff)
      {
        result->large = state->offset;
        return flash_not_erased;
      }

      // If the value is 0xffffffff, don't bother programming it
      if (state->buffer32 != 0xffffffff)
      {
        int ret;
        ret = flashProgram(state->buffer, (uint8_t *)state->offset, sizeof(uint32_t));
        if (ret != F_ERR_OK)
        {
          result->medium = ret;
          result->large = state->offset;
          return subsystem_error;
        }
      }

      state->count -= 4;
      state->offset += 4;

      state->buffer_offset = 0;
    }
  }

  return no_error;
}

static int do_echo_back(struct bl_state *state, struct result_pkt *result,
                        const struct bl_pkt *packet)
{
  (void)state;
  const uint8_t *src = (const uint8_t *)packet;
  uint8_t *dst = (uint8_t *)result;
  int i;
  for (i = 0; i < 7; i++)
    dst[i] = src[i + 1];
  return no_error;
}

static int do_hash_memory(struct bl_state *state, struct result_pkt *result,
                          const struct bl_pkt *packet)
{
  (void)state;
  uint32_t start = packet->hash.offset;
  uint32_t length = packet->hash.size;

  if (start & 3)
    return address_not_valid;

  if (length & 3)
    return size_not_valid;

  MurmurHash3_x86_32((const void *)start, length, 0, &result->large);

  return no_error;
}

static int do_bootloader_info(struct bl_state *state, struct info_pkt *result)
{
  (void)state;
  extern uint32_t __app_start__;
  result->block_size = 10;
  result->bl_enter_reason = bootloader_reason;
  result->bl_version = 2;
  result->app_offset = ((uint32_t)&__app_start__) / 1024;
  return no_error;
}

static int do_reboot(struct bl_state *state, struct result_pkt *result,
                     const struct reboot_pkt *rb)
{

  (void)state;

  result->medium = 0;

  if (rb->reboot_key[result->medium++] != 0x91)
    return key_not_valid;

  if (rb->reboot_key[result->medium++] != 0x82)
    return key_not_valid;

  if (rb->reboot_key[result->medium++] != 0x73)
    return key_not_valid;

  if (rb->reboot_key[result->medium++] != 0x64)
    return key_not_valid;

  if (rb->reboot_key[result->medium++] != 0xad)
    return key_not_valid;

  if (rb->reboot_key[result->medium++] != 0xef)
    return key_not_valid;

  if (rb->reboot_key[result->medium++] != 0xba)
    return key_not_valid;

  asm("bkpt #0");
  return no_error;
}

static int process_one_packet(struct bl_state *state,
                              struct bl_pkt *result,
                              const struct bl_pkt *packet)
{
  extern uint32_t __app_start__, __app_end__;
  switch (packet->cmd & 0x0f)
  {

  case bootloader_info:
    return do_bootloader_info(state, &result->info);

  case erase_block:
    return do_erase_flash(state, &result->result,
                          packet->erase_sector.offset, packet->erase_sector.count);

  case reboot_cmd:
    return do_reboot(state, &result->result, &packet->reboot);

  case start_programming:
    return do_start_programming(state, &result->result, &packet->start_programming);

  case program_data:
    return do_program_data(state, &result->result, &packet->program);

  case erase_app:
    return do_erase_flash(state, &result->result,
                          ((uint32_t)&__app_start__) / FTFx_PSECTOR_SIZE,
                          (((uint32_t)&__app_end__) - ((uint32_t)&__app_start__)) / FTFx_PSECTOR_SIZE);

  case peek_poke_cmd:
    return do_peek_poke(state, &result->result, &packet->read_write);

  case echo_back_cmd:
    return do_echo_back(state, &result->result, packet);

  case hash_memory:
    return do_hash_memory(state, &result->result, packet);

  default:
    asm("bkpt #43");
    return unhandled_command;
  }
}

int hits;
int misses;
int updateRx(void)
{
  struct bl_state state = {0};
  struct bl_pkt result_pkt = {0};
  int last_packet_num = -1;
  uint8_t last_cmd = 0;

  state.flash_is_protected = 1;

  /* Unlock PORTA and PORTB */
  SIM->SCGC5 |= SIM_SCGC5_PORTA | SIM_SCGC5_PORTB;

  /* Set up D+ and D- as slow-slew GPIOs (pin mux type 1), and enable IRQs */
  PORTB->PCR[5] = (1 << 8) | (0xb << 16) | (1 << 2);
  PORTB->PCR[6] = (1 << 8) | (0xb << 16) | (1 << 2);

  grainuumInit(&defaultUsbPhy, &hid_link);
  grainuumDisconnect(&defaultUsbPhy);

  {
    int i;
    for (i = 0; i < 1000; i++)
    {
      int j;
      for (j = 0; j < 77; j++)
      {
        asm("");
      }
    }
  }

  /* Enable PORTB IRQ */
  NVIC_EnableIRQ(PINB_IRQn);
  __enable_irq();

  grainuumConnect(&defaultUsbPhy);

  while (!done)
  {
    process_all_usb_events(&defaultUsbPhy);

    // If the rx_buffer_head has advnaced, then we have data to process in EP1
    if (rx_buffer_head != rx_buffer_tail)
    {
      struct bl_pkt *incoming_pkt = (struct bl_pkt *)rx_buffer[rx_buffer_tail];

      // If the last packet num is the same as this packet num, then we've processed
      // this packet already, but were unsuccessful in sending the packet.  If not,
      // as is the case here, handle it as a new packet.
      if (rx_buffer_tail != last_packet_num)
      {
        last_packet_num = rx_buffer_tail;
        last_cmd = incoming_pkt->cmd;

        // Pack the sequence number and the 'result' command together
        result_pkt.cmd = result_cmd | (last_cmd & 0xf0);

        unsigned int i;
        for (i = 0; i < sizeof(result_pkt.raw.data); i++)
          result_pkt.raw.data[i] = 0;

        result_pkt.result.small = process_one_packet(&state, &result_pkt, incoming_pkt);
        if (state.continue_function)
          result_pkt.cmd = ongoing_process_cmd | (last_cmd & 0xf0);
      }

      // Advance the packet number only if this packet was sent successfully
      if (!grainuumSendData(&defaultUsbPhy, 1, &result_pkt, sizeof(result_pkt)))
      {
        rx_buffer_tail = (rx_buffer_tail + 1) & (NUM_BUFFERS - 1);
      }
    }

    // If there is an ongoing process, run that function.
    if (state.continue_function != NULL)
    {

      result_pkt.cmd = ongoing_process_cmd | (last_cmd & 0xf0);
      grainuumDropData(&defaultUsbPhy);
      grainuumSendData(&defaultUsbPhy, 1, &result_pkt, sizeof(result_pkt));

      // If the function returns zero, deregister it and send "finish".
      if (!state.continue_function(&state, &result_pkt.result, state.continue_arg))
      {
        result_pkt.cmd = result_cmd | (last_cmd & 0xf0);
        state.continue_function = NULL;
      }

      grainuumDropData(&defaultUsbPhy);
      if (grainuumSendData(&defaultUsbPhy, 1, &result_pkt, sizeof(result_pkt)))
        misses++;
      else
        hits++;
    }
  }

  return 0;
}
