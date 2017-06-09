#ifndef __ORCHARD_RADIO_H__
#define __ORCHARD_RADIO_H__

#include <stddef.h>
#include <stdint.h>
#include "radio-prot.h"

struct _KRadioDevice;
typedef struct _KRadioDevice KRadioDevice;

extern KRadioDevice KRADIO1;
#define radioDevice &KRADIO1

#define RADIO_NETWORK_MAX_LENGTH 8
#define RADIO_BROADCAST_ADDRESS 255

typedef struct _RadioPacket {
  uint8_t length;       /* Total length of packet, including length field */
  uint8_t dst;          /* Address of intended recipient */
  uint8_t src;          /* Address of device sending packet */
  uint8_t prot;         /* Subsystem packet is intended for */
  uint8_t payload[0];   /* Actual contents of packet */
} RadioPacket;

extern KRadioDevice KRADIO1;

void radioInit(void);
void radioStart(KRadioDevice *radio);
void radioStop(KRadioDevice *radio);
void radioPoll(KRadioDevice *radio);
uint8_t radioRead(KRadioDevice *radio, uint8_t addr);
void radioWrite(KRadioDevice *radio, uint8_t addr, uint8_t val);
int radioDump(KRadioDevice *radio, uint8_t addr, void *bfr, int count);
int radioTemperature(KRadioDevice *radio);
void radioSetNetwork(KRadioDevice *radio, const uint8_t *id, uint8_t len);
void radioSend(KRadioDevice *radio, uint8_t dest, uint8_t prot,
                                    size_t len, const void *payload);
void radioSetAddress(KRadioDevice *radio, uint8_t addr);
uint8_t radioAddress(KRadioDevice *radio);

void radioSetDefaultHandler(KRadioDevice *radio,
                            void (*handler)(uint8_t prot,
                                            uint8_t src,
                                            uint8_t dst,
                                            uint8_t length,
                                            const void *data));
void radioSetHandler(KRadioDevice *radio, uint8_t prot,
                     void (*handler)(uint8_t prot,
                                     uint8_t src,
                                     uint8_t dst,
                                     uint8_t length,
                                     const void *data));

#endif /* __ORCHARD_RADIO_H__ */
