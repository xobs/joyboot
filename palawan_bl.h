#ifndef __PALAWAN_B_H__
#define __PALAWAN_B_H__

#define PACKED __attribute__((packed))

struct reboot_pkt {
  uint8_t reboot_key[7]; // Must be 0x91 0x82 0x73 0x64 0xad 0xef 0xba
} PACKED;

struct raw_pkt {
  uint8_t data[7];
} PACKED;

struct info_pkt {
  uint8_t   result;           // Result error code
  uint16_t  block_size;       // Block size (in bits, e.g. a size of '1024' results in '10')
  uint8_t   bl_enter_reason;  // The reason we entered the bootloader
  uint8_t   bl_version;       // Version of the bootloader
  uint16_t  reserved2;
} PACKED;

struct erase_sector_pkt {
  uint8_t  reserved;
  uint16_t count;     // Number of sectors to erase
  uint32_t offset;    // First sector to erase
} PACKED;

struct start_programming_pkt {
  uint8_t  reserved;
  uint16_t count;     // Number of bytes to program
  uint32_t offset;    // Offset where program will start
} PACKED;

struct program_pkt {
  uint8_t  data[7];   // Bytes to program
} PACKED;

struct hash_pkt {
  uint8_t  reserved;
  uint16_t size;      // Number of bytes to hash
  uint32_t offset;    // Address to start hashing from
} PACKED;

struct read_write_pkt {
  uint8_t  type; // xRPPooSS
    // SS: Size of operation
    //  00 - 8-bit
    //  01 - 16-bit
    //  10 - 32-bit
    //  11 - Dummy operation -- No read/write/increment is done, but address is set
    #define PEEK_POKE_SIZE_MASK (3 << 0)
    #define PEEK_POKE_SIZE_8    (0 << 0)
    #define PEEK_POKE_SIZE_16   (1 << 0)
    #define PEEK_POKE_SIZE_32   (2 << 0)
    #define PEEK_POKE_SIZE_DUMMY (3 << 0)
    // oo: Operation
    #define PEEK_POKE_OP_MASK (3 << 2)
    #define PEEK_POKE_OP_NONE (0 << 2)
    #define PEEK_POKE_OP_SET  (1 << 2)
    #define PEEK_POKE_OP_CLR  (2 << 2)
    #define PEEK_POKE_OP_TOG  (3 << 2)
    //  00 - no operation
    //  01 - set bits
    //  10 - clear bits
    //  11 - toggle bits
    // PP: Modify/increment pointer
    #define PEEK_POKE_INCR_MASK (3 << 4)
    #define PEEK_POKE_INCR (2 << 4)
    #define PEEK_POKE_DECR (3 << 4)
    //  00 - Do not increment
    //  01 - Reserved
    //  10 - Increment pointer after operation
    //  11 - Decrement pointer after operation
    // R: 1 for read, 0 for write
    #define PEEK_POKE_READ (1 << 6)
  uint16_t increment;
    // c: Number of units to increment/decrement, minus one
    //  0 - Increment/decrement by 1 (i.e. for a 32-bit read, increment/decrement by 4 bytes)
    //  1 - Increment/decrement by 2
    //  ...
  union {
    uint32_t value;     // For a 'write', uses the address from the last 'read'
    uint32_t address;   // For a 'read', this is the address
  };
} PACKED;

struct jump_to_address_pkt {
  uint8_t  reserved[3];
  uint32_t address;
} PACKED;

struct ongoing_pkt {
  uint8_t  cmd;
  uint16_t reserved;
  uint32_t progress;
} PACKED;

struct result_pkt {
  uint8_t  small;
  uint16_t medium;
  uint32_t large;
} PACKED;

struct bl_pkt {
  uint8_t cmd; // Upper four bits are sequence number.
  union {
    struct raw_pkt raw;
    struct info_pkt info;
    struct erase_sector_pkt erase_sector;
    struct start_programming_pkt start_programming;
    struct program_pkt program;
    struct hash_pkt hash;
    struct read_write_pkt read_write;
    struct jump_to_address_pkt jump_to_address;
    struct reboot_pkt reboot;
    struct ongoing_pkt ongoing;
    struct result_pkt result;
  } PACKED;
} __attribute__((packed, aligned(4)));

#undef PACKED

enum bl_pkt_command {
  bootloader_info = 0,
  erase_block = 1,
  erase_app = 2,
  start_programming = 3,
  program_data = 4,
  hash_memory = 5,
  jump_cmd = 6,
  peek_poke_cmd = 8,
  echo_back_cmd = 9,
  reboot_cmd = 10,
  ongoing_process_cmd = 14,
  result_cmd = 15,
};

enum bl_pkt_result {
  no_error = 0,
  unhandled_command = 1,
  address_out_of_range = 2,
  no_address_set = 3,
  subsystem_error = 4,
  address_not_valid = 5,
  size_not_valid = 6,
  key_not_valid = 7,
  flash_not_erased = 8,
  command_ongoing = 9,
};

struct bl_state {
  uint32_t offset;
  uint32_t count;
  int (*continue_function)(struct bl_state *state, struct result_pkt *result, void *arg);
  void *continue_arg;
  union {
    uint8_t buffer[4];
    uint32_t buffer32;
  };
  uint8_t buffer_offset;
  uint8_t flash_is_protected;
};

enum bootloader_reason {
  NOT_ENTERING_BOOTLOADER = 0,
  BOOT_TOKEN_PRESENT = 1,
  BOOT_FAILED_TOO_MANY_TIMES = 2,
  NO_PROGRAM_PRESENT = 3,
  BUTTON_HELD_DOWN = 4,
};

extern enum bootloader_reason bootloader_reason;

#endif /* __PALAWAN_BL_H__ */
