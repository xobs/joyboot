/*
 * Update the firmware over Radio.
 */

#include "kl17.h"
#include "palawan.h"
#include "radio.h"
#include "murmur3.h"

#define REQUEST_FIRMWARE_DATA 64

int dhcpRequestAddress(int timeout_ms);

__attribute__((aligned(4)))
static uint8_t one_page[1024];

#define MURMUR_REQUEST  0x5592493f
#define MURMUR_RESPONSE 0x239b23a9
#define MURMUR_SEED     0xb62a238c

struct firmware_request {
  uint32_t type;
  uint32_t offset;
  uint32_t length;
  uint32_t bytes_per_packet;
} __attribute__((packed));

struct firmware_murmur_response {
  uint32_t type;
  uint32_t payload_size;
  uint32_t offset;
  uint8_t payload[32];
  uint32_t murmur_sum;
} __attribute__((packed));


static const uint32_t app_flash_start = 0x00002800;
static const uint32_t app_flash_end = 0x00002800 + (1024 * 118);

static struct firmware_murmur_response response;
static int response_valid;

static void prot_get_firmware(uint8_t port,
                              uint8_t src,
                              uint8_t dst,
                              uint8_t length,
                              const void *data) {

  const struct firmware_murmur_response *r = data;
  uint32_t chk;

  if (r->type != MURMUR_RESPONSE)
    goto invalid_response;

  if (length != sizeof(*r))
    goto invalid_response;

  MurmurHash3_x86_32(data, sizeof(*r) - 4, MURMUR_SEED, &chk);

  if (chk != r->murmur_sum)
    goto invalid_response;

  int i;
  for (i = 0; i < length; i++)
    ((uint8_t *)&response)[i] = ((uint8_t *)data)[i];
  response_valid = 1;

  return;

invalid_response:
  return;
}

static int get_one_page(KRadioDevice *radio, uint8_t data[1024], int pagenum) {

  struct firmware_request request = {
    .type             = MURMUR_REQUEST,
    .offset           = (pagenum * 1024),
    .length           = 1024,
    .bytes_per_packet = 32,
  };

  /* Send a request for the data, and wait for it to come back */
  radioSend(radio, 0, REQUEST_FIRMWARE_DATA, sizeof(request), &request);

  /*
  while (i < 1024) {
    uint32_t addr = (pagenum * 1024) + i;

  }
  */

  return 0;
}

int updateTx(void) {

  int pagenum;

  /* PTC4 goes high when a packet is available */
  PORTC->PCR[4] = PORTx_PCRn_MUX(1);
  GPIOC->PDDR &= ~((uint32_t) 1 << 4);

  radioStart(radioDevice);

  while (dhcpRequestAddress(1000) < 0)
    ;

  radioSetHandler(radioDevice, REQUEST_FIRMWARE_DATA, prot_get_firmware);

  for (pagenum = (app_flash_start / 1024); pagenum < (app_flash_end / 1024); pagenum++) {
    get_one_page(radioDevice, one_page, pagenum);
  }

  return 0;
}
