#ifndef __PALAWAN_B_H__
#define __PALAWAN_B_H__

#define PACKED __attribute__((packed))

struct raw_pkt {
  uint8_t data[6];
} PACKED;

struct erase_sector_pkt {
  uint16_t count;
  uint32_t offset;
} PACKED;

struct set_address_pkt {
  uint8_t reserved[2];
  uint32_t offset;
} PACKED;

struct program_pkt {
  uint8_t reserved[2];
  uint32_t data;
} PACKED;

struct hash_pkt {
  uint16_t size;
  uint32_t offset;
} PACKED;

struct result_pkt {
  uint8_t code;
  uint8_t extra;
  uint8_t reserved[4];
} PACKED;

struct bl_pkt {
  uint8_t seq_num;
  uint8_t cmd;
  union {

    struct raw_pkt raw;
    struct erase_sector_pkt erase_sector;
    struct set_address_pkt set_address;
    struct program_pkt program;
    struct hash_pkt hash;
    struct result_pkt result;

  } PACKED;
} __attribute__((packed, aligned(4)));
#undef PACKED

enum bl_pkt_command {
  erase_block = 1,
  set_address = 2,
  program_value = 3,
  hash_memory = 4,
  echo_back = 5,
  packet_result = 255,
};

enum bl_pkt_result {
  no_error = 0,
  unhandled_command = 1,
  address_out_of_range = 2,
  no_address_set = 3,
  subsystem_error = 4,
  address_not_valid = 5,
  size_not_valid = 6,
};

struct bl_state {
  uint32_t offset;
  uint32_t flash_is_protected;
};

#endif /* __PALAWAN_BL_H__ */