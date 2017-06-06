#include <stdint.h>
#include "radio.h"
#include "palawan.h"

struct dhcp_request {
  uint8_t addr;
  uint64_t uid;
} __attribute__((packed));

static long long get_uid(void) {
  long long uidh = *((uint32_t *)0x40048058);
  long long uidm = *((uint32_t *)0x4004805c);
  long long uidl = *((uint32_t *)0x40048060);
  long long uid;

  uid = (uidh << 48) | (uidm << 16) | (uidl >> 16);
  return uid;
}

static uint8_t dhcp_requesting;

static void prot_dhcp(uint8_t port,
                      uint8_t src,
                      uint8_t dst,
                      uint8_t length,
                      const void *data) {
  (void)port;
  (void)dst;
  (void)length;
  const struct dhcp_request *req = data;

  /* Client code (e.g. we got a response) */
  if (req->uid == get_uid()) {

    radioSetAddress(radioDevice, req->addr);
    dhcp_requesting = 0;
  }

  return;
}

int dhcpRequestAddress(int timeout_ms) {

  struct dhcp_request request;
  int ms = 0;

  request.uid = get_uid();
  request.addr = 0;

  dhcp_requesting = 1;
	radioSetHandler(radioDevice, 3, prot_dhcp);
  radioSend(radioDevice, 0xff, 3, sizeof(request), &request);

  while (dhcp_requesting && (ms < timeout_ms)) {
		radioPoll(radioDevice);
		int i;
		for (i = 0; i < 17000; i++)
			asm("nop");
    ms++;
  }

  if (dhcp_requesting)
    ms = -1;
  dhcp_requesting = 0;

  return ms;
}
