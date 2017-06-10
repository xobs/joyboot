#ifndef _RADIO_PROTOCOLS_H
#define _RADIO_PROTOCOLS_H

enum radio_protocols {
  radio_prot_paging      = 1,
  radio_prot_dhcp_request     = 2,
  radio_prot_dhcp_response    = 3,
  radio_prot_echo_test        = 4,
  radio_prot_dut_to_peer = 6,
  radio_prot_peer_to_dut = 7,
};

#endif /* _RADIO_PROTOCOLS_H */
