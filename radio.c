
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "kl17.h"
#include "radio.h"
#include "TransceiverReg.h"
#include "spi.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) ((sizeof(x)) / (*x))
#endif

#define REG_TEMP1                 0x4e
#define REG_TEMP1_START             (1 << 3)
#define REG_TEMP1_RUNNING           (1 << 2)
#define REG_TEMP2                 0x4f

#define RADIO_XTAL_FREQUENCY      32000000 /* 32 MHz crystal */
#define RADIO_FIFO_DEPTH          66
#define RADIO_BUFFER_SIZE         64
#define RADIO_BUFFER_MASK         (RADIO_BUFFER_SIZE - 1)

#define MAX_PACKET_HANDLERS       10

/* This number was guessed based on observations (133 at 30 degrees) */
static int temperature_offset = 133 + 30;

enum modulation_type {
  modulation_fsk_no_shaping = 0,
  modulation_fsk_gaussian_bt_1p0 = 1,
  modulation_fsk_gaussian_bt_0p5 = 2,
  modulation_fsk_gaussian_bt_0p3 = 3,
  modulation_ook_no_filter = 8,
  modulation_ook_filter_br = 9,
  modulation_ook_filter_2xbr = 10,
};

enum radio_mode {
  mode_sleep,
  mode_standby,
  mode_fs,
  mode_receiving,
  mode_transmitting,
};

enum encoding_type {
  encoding_none = 0,
  encoding_manchester = 1,
  encoding_whitening = 2,
};

typedef struct _PacketHandler {
    void (*handler)(uint8_t prot, uint8_t src, uint8_t dst,
                    uint8_t length, const void *data);
      uint8_t prot;
} PacketHandler;

/* Kinetis Radio definition */
typedef struct _KRadioDevice {
  uint16_t                bit_rate;
  uint32_t                channel;
  uint8_t                 rx_buf[RADIO_BUFFER_SIZE];
  uint32_t                rx_buf_end;
  uint32_t                rx_buf_pos;
  uint8_t                 address;
  uint8_t                 broadcast;
  uint8_t                 num_handlers;
  PacketHandler           handlers[MAX_PACKET_HANDLERS];
  void                    (*default_handler)(uint8_t prot,
                                             uint8_t src,
                                             uint8_t dst,
                                             uint8_t length,
                                             const void *data);
  enum modulation_type    modulation;
  enum radio_mode         mode;
  enum encoding_type      encoding;
} KRadioDevice;

KRadioDevice KRADIO1;
#define radioDevice &KRADIO1

static uint8_t const default_registers[] = {
  /* Radio operation mode initialization @0x01*/
  RADIO_OpMode, OpMode_Sequencer_On | OpMode_Listen_Off | OpMode_StandBy,

  /* Radio Data mode and modulation initialization @0x02*/
//  RADIO_DataModul, DataModul_DataMode_Packet | DataModul_Modulation_Fsk | DataModul_ModulationShaping_BT_05,

  /* Radio bit rate initialization @0x03-0x04*/
  RADIO_BitrateMsb, BitrateMsb_4800,
  RADIO_BitrateLsb, BitrateLsb_4800,

  /* Radio frequency deviation initialization @0x05-0x06*/
  RADIO_FdevMsb, FdevMsb_2400,
  RADIO_FdevLsb, FdevLsb_2400,

  /* Radio RF frequency initialization @0x07-0x09*/
  /*Default Frequencies*/
#define DEFAULT_FRF_434

#ifdef DEFAULT_FRF_915
  RADIO_FrfMsb, FrfMsb_915,
  RADIO_FrfMid, FrfMid_915,
  RADIO_FrfLsb, FrfLsb_915,
#endif

#ifdef DEFAULT_FRF_868
  RADIO_FrfMsb, FrfMsb_868,
  RADIO_FrfMid, FrfMid_868,
  RADIO_FrfLsb, FrfLsb_868,
#endif

#ifdef DEFAULT_FRF_865
  RADIO_FrfMsb, FrfMsb_865,
  RADIO_FrfMid, FrfMid_865,
  RADIO_FrfLsb, FrfLsb_865,
#endif

#ifdef DEFAULT_FRF_470
  RADIO_FrfMsb, FrfMsb_470,
  RADIO_FrfMid, FrfMid_470,
  RADIO_FrfLsb, FrfLsb_470,
#endif

#ifdef DEFAULT_FRF_434
  RADIO_FrfMsb, FrfMsb_434,
  RADIO_FrfMid, FrfMid_434,
  RADIO_FrfLsb, FrfLsb_434,
#endif

#ifdef DEFAULT_FRF_920                                                          //JAPAN
  RADIO_FrfMsb, FrfMsb_920,
  RADIO_FrfMid, FrfMid_920,
  RADIO_FrfLsb, FrfLsb_920,
#endif

  /* Radio RegAfcCtrl initialization @0x0B*/
  RADIO_AfcCtrl, AfcCtrl_AfcLowBeta_Off ,

  /* Radio output power initialization @0x11*/
  RADIO_PaLevel, PaLevel_Pa0_On | PaLevel_Pa1_Off | PaLevel_Pa2_Off | 0x1F,

  /* Radio Rise/Fall time of ramp up/down in FSK initialization @0x12*/
  RADIO_PaRamp, PaRamp_40,

  /* Radio overload current protection for PA initialization 0x13*/
  RADIO_Ocp, Ocp_Ocp_On | 0x0C,

  /* Radio LNA gain and input impedance initialization @0x18*/
  RADIO_Lna, Lna_LnaZin_50 | Lna_LnaGain_Agc,

  /* Radio channel filter bandwidth initialization @0x19*/
  RADIO_RxBw, DccFreq_7 | RxBw_10400,

  /* Radio channel filter bandwidth for AFC operation initialization @0x1A*/
  RADIO_AfcBw, DccFreq_7 | RxBw_10400,

  /* Radio automatic frequency control initialization @0x1E*/
  RADIO_AfcFei, AfcFei_AfcAuto_Off | AfcFei_AfcAutoClear_On,

  /* Radio Rssi threshold initialization @0x29*/
  // RSSIthres = [-174 + NF +10*log(2*RxBw) + DemodSNR] dBm
  // NF = 7dB
  // DemodSnr = 8dB
  // RxBw depends on frequency bands and profiles
  RADIO_RssiThresh, 0xDC, // -101 dBm for 333.3 Khz singleside channel filter bandwith

  /* Radio RegTimeoutRxStart initialization @0x2A*/
  /* Radio RegTimeoutRssiThresh initialization @0x2B*/
  RADIO_RxTimeout1, 0x00, //disable timeout rx start
  RADIO_RxTimeout2, 0x00, //disable timeout rx start

  /* MKW01 preamble size initialization @0x2C-0x2D*/
  RADIO_PreambleMsb, 0x00,
  RADIO_PreambleLsb, 0x10,

  /* Radio sync word control and value initialization @0x2E-0x30*/
  RADIO_SyncConfig, SyncConfig_Sync_On | SyncConfig_FifioFill_ifSyncAddres | SyncConfig_SyncSize_2,
  RADIO_SyncValue1, 0x90, //SFD value for uncoded with phySUNMRFSKSFD = 0
  RADIO_SyncValue2, 0x4E, //SFD value for uncoded with phySUNMRFSKSFD = 0

  /* Radio packet mode config */
  RADIO_PacketConfig1, PacketConfig1_PacketFormat_Variable_Length | PacketConfig1_AddresFiltering_Node_Or_Broadcast | PacketConfig1_Crc_On,
  RADIO_PacketConfig2, 0x00,

  /* Radio payload length initialization */
  RADIO_PayloadLength, 255,  //max length in rx

  /* Prep a temperature sample */
  RADIO_Temp1, REG_TEMP1_START,
};

static void spiSend(void *ignored, int count, const void *data) {
  (void)ignored;
  int i;
  const uint8_t *bytes = data;

  for (i = 0; i < count; i++)
    spiXmitByteSync(bytes[i]);

  /* Sync byte */
  //spiXmitByteSync(0xff);
}

static void spiSync(void *ignored) {
  (void)ignored;
  spiXmitByteSync(0xff0);
}

static void spiReceive(void *ignored, int count, void *data) {
  (void)ignored;

  int i;
  uint8_t *bytes = data;

  for (i = 0; i < count; i++)
    bytes[i] = spiRecvByteSync();
}

static void radio_select(KRadioDevice *radio) {
  (void)radio;
  spiAssertCs();
}

static void radio_unselect(KRadioDevice *radio) {
  (void)radio;
  spiDeassertCs();
}

static void radio_set(KRadioDevice *radio, uint8_t addr, uint8_t val) {

  uint8_t buf[2] = {addr | 0x80, val};

  radio_select(radio);
  spiSend(NULL, 2, buf);
  radio_unselect(radio);
}

static uint8_t radio_get(KRadioDevice *radio, uint8_t addr) {

  uint8_t val;

  radio_select(radio);
  spiSend(NULL, 1, &addr);
  spiSync(NULL);
  spiReceive(NULL, 1, &val);
  radio_unselect(radio);
  return val;
}

void radioPhySetBitRate(KRadioDevice *radio, uint32_t rate) {

  rate = RADIO_XTAL_FREQUENCY / rate;
  radio_set(radio, RADIO_BitrateMsb, rate >> 8);
  radio_set(radio, RADIO_BitrateLsb, rate);
}

static void radio_phy_update_modulation_parameters(KRadioDevice *radio) {

 /* Radio channel filter bandwidth initialization @0x19*/
  radio_set(radio, RADIO_RxBw, DccFreq_3 | RxBw_83300);

  /* Radio channel filter bandwidth for AFC operation initialization @0x1A*/
  radio_set(radio, RADIO_AfcBw, DccFreq_3 | RxBw_83300);
}

void radioPhySetRfDeviation(KRadioDevice *radio, uint32_t fdev)
{
  /* Calculation->  Frf = Foperate/Fstep
   *                Fstep = (Fxosc / 2**19)
   *                (Fstep = 61.03515625; for a 32Mhz FXOSC)
   *                Fstep * 256 = 15625.0
   */
  const uint32_t Fstep_32 = 15625; /* Fstep at 32 MHz times 256 */

  fdev *= 256;
  fdev /= Fstep_32;

  radio_set(radio, RADIO_FdevMsb, fdev >> 8);
  radio_set(radio, RADIO_FdevLsb, fdev);
}

uint32_t radioPhyRfDeviation(KRadioDevice *radio)
{
  const uint32_t Fstep_32 = 15625; /* Fstep at 32 MHz times 256 */
  uint32_t fdev;

  fdev  = (radio_get(radio, RADIO_FdevMsb) << 8) & 0x3f00;
  fdev |= (radio_get(radio, RADIO_FdevLsb)) & 0xff;;
  fdev *= Fstep_32;
  fdev /= 256;

  return fdev;
}

void radioPhyUpdateRfFrequency(KRadioDevice *radio, uint32_t freq) {

 uint32_t channel_frequency;

  /* Calculation->  Frf = Foperate/Fstep
   *                Fstep = (Fxosc / 2**19)
   *                (Fstep = 61.03515625; for a 32Mhz FXOSC)
   *                Fstep * 256 = 15625.0
   */
  const uint32_t Fstep_32 = 15625; /* Fstep at 32 MHz times 256 */
  uint32_t Frf = freq;

#warning "Verify this works"
  Frf /= Fstep_32;
  Frf *= 256;

  /* This value corresponds to a channel spacing of 1 MHz.*/
  channel_frequency = radio->channel * 0x4000;

  /* This value corresponds to a Operating Frequency of 902,500,000
   * Value for Channel0 (902.500 MHz)
   */
  channel_frequency += Frf;

  radio_set(radio, RADIO_FrfMsb, channel_frequency >> 16);
  radio_set(radio, RADIO_FrfMid, channel_frequency >> 8);
  radio_set(radio, RADIO_FrfLsb, channel_frequency);
}

static void radio_set_preamble_length(KRadioDevice *radio, uint32_t length) {

  /* Radio preamble size initialization @0x2C-0x2D*/
  radio_set(radio, RADIO_PreambleMsb, length >> 8);
  radio_set(radio, RADIO_PreambleLsb, length);
}

/* Valid range: -18 to 13 dBm */
static void radio_set_output_power_dbm(KRadioDevice *radio, int power) {

  radio_set(radio, RADIO_PaLevel, PaLevel_Pa0_On
                                | PaLevel_Pa1_Off
                                | PaLevel_Pa2_Off
                                | ((power + 18) & 0x1f));
}

#if 0
static void radio_phy_force_idle(KRadioDevice *radio) {
  //Put transceiver in Stand-By mode
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                            | OpMode_Listen_Off
                            | OpMode_StandBy);

  //clear the transceiver FIFO
  while(radio_get(radio, RADIO_IrqFlags2) & 0x40)
    (void)radio_get(radio, RADIO_Fifo);
}
#endif

static void radio_set_packet_mode(KRadioDevice *radio) {
  uint8_t reg;

  reg = radio_get(radio, RADIO_DataModul);
  reg &= ~DataModul_DataMode_Mask;
  reg |= DataModul_DataMode_Packet;
  radio_set(radio, RADIO_DataModul, reg);
}

static void radio_set_modulation(KRadioDevice *radio,
                                 enum modulation_type modulation) {

  uint8_t reg;

  reg = radio_get(radio, RADIO_DataModul);
  reg &= ~DataModul_ModulationShaping_Mask;
  reg &= ~DataModul_Modulation_Mask;
  reg |= modulation;
  radio_set(radio, RADIO_DataModul, reg);
}

void radio_set_encoding(KRadioDevice *radio, enum encoding_type encoding) {

  uint8_t reg;

  radio->encoding = encoding;
  reg = radio_get(radio, RADIO_PacketConfig1);
  reg &= ~PacketConfig1_DcFree_Mask;
  reg |= (encoding << PacketConfig1_DcFree_Shift);
  radio_set(radio, RADIO_PacketConfig1, reg);
}

static void radio_set_node_address(KRadioDevice *radio, uint8_t address) {

  radio->address = address;
  radio_set(radio, RADIO_NodeAddress, address);
}

static void radio_set_broadcast_address(KRadioDevice *radio, uint8_t address) {

  radio->broadcast = address;
  radio_set(radio, RADIO_BroadcastAddress, address);
}

void radioUnloadPacket(KRadioDevice *radio) {

  RadioPacket pkt;
  uint8_t reg, crc;

  radio_select(radio);
  reg = RADIO_Fifo;
  spiSend(NULL, 1, &reg);
  spiSync(NULL);
  
  /* Read the "length" byte */
  spiReceive(NULL, sizeof(pkt), &pkt);

  uint8_t payload[pkt.length - sizeof(pkt)];

  /* read the remainder of the packet */
  spiReceive(NULL, sizeof(payload), payload);
  spiReceive(NULL, sizeof(crc), &crc);
  radio_unselect(radio);

  /* Dispatch the packet handler */
  unsigned int i;
  bool handled = false;
  for (i = 0; i < radio->num_handlers; i++) {
    if (radio->handlers[i].prot == pkt.prot) {
      radio->handlers[i].handler(pkt.prot,
                                 pkt.src,
                                 pkt.dst,
                                 sizeof(payload),
                                 payload);
      handled = true;
      break;
    }
  }

  /* If the packet wasn't handled, pass it to the default handler */
  if (!handled && radio->default_handler)
      radio->default_handler(pkt.prot,
                             pkt.src,
                             pkt.dst,
                             sizeof(payload),
                             payload);
}

void radioPoll(KRadioDevice *radio) {

  if (!(GPIOB->PDIR & (1 << 2)))
    return;

  radioUnloadPacket(radio);
}

void radioStop(KRadioDevice *radio) {
  radio_set(radio, RADIO_OpMode, 0x80); // force into sleep mode immediately
}

void radioStart(KRadioDevice *radio) {

  unsigned int reg;

  reg = 0;
  while (reg < ARRAY_SIZE(default_registers)) {
    uint8_t cmd = default_registers[reg++];
    uint8_t dat = default_registers[reg++];

    radio_set(radio, cmd, dat);
  }

  radio_phy_update_modulation_parameters(radio);
  //radioPhySetBitRate(radio, 50000);
  //radioPhySetRfDeviation(radio, 26370);
  //radioPhySetRfDeviation(radio, 40625/2);
  //radioPhySetBitRate(radio, 40625);

  radio->channel = 0;
  //radioPhyUpdateRfFrequency(radio, 433923000);

  radio_set_preamble_length(radio, 3);

  radio_set_output_power_dbm(radio, 13); /* Max output with PA0 is 13 dBm */

  radio_set_encoding(radio, encoding_whitening);
  radio_set_modulation(radio, modulation_fsk_no_shaping);
  radio_set_packet_mode(radio);
  radio_set_broadcast_address(radio, 255);
  radio_set_node_address(radio, 1);

  /* Drain the Fifo */
  while (radio_get(radio, RADIO_IrqFlags2) & IrqFlags2_FifoNotEmpty)
    (void)radio_get(radio, RADIO_Fifo);

  /* Move into "Rx" mode */
  radio->mode = mode_receiving;
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                               | OpMode_Listen_Off
                               | OpMode_Receiver);
}

void radioSetHandler(KRadioDevice *radio,
                     uint8_t prot,
                     void (*handler)(uint8_t prot,
                                     uint8_t src,
                                     uint8_t dst,
                                     uint8_t length,
                                     const void *data)) {
  unsigned int i;

  /* Replace an existing handler? */
  for (i = 0; i < radio->num_handlers; i++) {
    if (radio->handlers[i].prot == prot) {
      radio->handlers[i].handler = handler;
      return;
    }
  }

  radio->handlers[radio->num_handlers].prot    = prot;
  radio->handlers[radio->num_handlers].handler = handler;
  radio->num_handlers++;
}

void radioSetDefaultHandler(KRadioDevice *radio,
                            void (*handler)(uint8_t prot,
                                            uint8_t src,
                                            uint8_t dst,
                                            uint8_t length,
                                            const void *data)) {
  radio->default_handler = handler;
}

uint8_t radioRead(KRadioDevice *radio, uint8_t addr) {

  uint8_t val;

  val = radio_get(radio, addr);

  return val;
}

void radioWrite(KRadioDevice *radio, uint8_t addr, uint8_t val) {

  radio_set(radio, addr, val);
}

int radioDump(KRadioDevice *radio, uint8_t addr, void *bfr, int count) {

  radio_select(radio);
  spiSend(NULL, 1, &addr);
  spiSync(NULL);
  spiReceive(NULL, count, bfr);
  radio_unselect(radio);

  return 0;
}

int radioTemperature(KRadioDevice *radio) {

  uint8_t buf[2];

  radio_set(radio, REG_TEMP1, REG_TEMP1_START);

  do {
    buf[0] = REG_TEMP1;

    radio_select(radio);
    spiSend(NULL, 1, buf);
    spiSync(NULL);
    spiReceive(NULL, 2, buf);
    radio_unselect(radio);
  }
  while (buf[0] & REG_TEMP1_RUNNING);


  return (temperature_offset - buf[1]);
}

void radioSetNetwork(KRadioDevice *radio, const uint8_t *id, uint8_t len) {

  uint8_t reg;
  uint32_t ptr;

  reg = radio_get(radio, RADIO_SyncConfig);

  /* Disable sync config */
  if (!id || !len) {
    radio_set(radio, RADIO_SyncConfig, reg & ~RADIO_SyncConfig_SyncOn);
    return;
  }

  if (len > RADIO_NETWORK_MAX_LENGTH)
    len = RADIO_NETWORK_MAX_LENGTH;

  radio_set(radio, RADIO_SyncConfig, reg | RADIO_SyncConfig_SyncOn);
  for (ptr = 0; ptr < len; ptr++) {
    reg = id[ptr];

    /* NOTE: The radio does not allow sync words to contain 0 bytes (7.5.7.1) */
    if (reg == 0)
      reg = 1;

    radio_set(radio, RADIO_SyncValue1 + ptr, reg);
  }
}

void radioSetAddress(KRadioDevice *radio, uint8_t addr) {

  radio->address = addr;
  radio_set_node_address(radio, addr);
}

uint8_t radioAddress(KRadioDevice *radio) {

  return radio->address;
}

void radioSend(KRadioDevice *radio,
               uint8_t addr,
               uint8_t prot,
               size_t bytes,
               const void *payload) {

  RadioPacket pkt;
  uint8_t reg;

  pkt.length = bytes + sizeof(pkt);
  pkt.src = radio->address;
  pkt.dst = addr;
  pkt.prot = prot;

  /* Ideally, we'd poll for DIO1 to see when the FIFO can accept data.
   * This is not wired up on Orchard, so we can't transmit packets larger
   * than the FIFO.
   */
//  osalDbgAssert(pkt.length < RADIO_FIFO_DEPTH, "Packet is too large");

  /* Enter transmission mode */
  radio->mode = mode_transmitting;
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                               | OpMode_Listen_Off
                               | OpMode_Transmitter);

  /* Transmit the packet as soon as the entire thing is in the Fifo */
  radio_set(radio, RADIO_FifoThresh, pkt.length - 1);

  radio_select(radio);

  /* Select the FIFO */
  reg = RADIO_Fifo | 0x80;
  spiSend(NULL, 1, &reg);

  /* Load the header into the Fifo */
  spiSend(NULL, sizeof(pkt), &pkt);

  /* Load the payload into the Fifo */
  spiSend(NULL, bytes, payload);
  radio_unselect(radio);

  /* Wait for DIO0 to go high, indicating the transmission has finished */
  while (!(FGPIOA->PDIR & (1 << 8)))
    ;
  /* Wait for the transmission to complete (will be unlocked in IRQ) */
//  osalSysLock();
//  (void) osalThreadSuspendS(&radio->thread);
//  osalSysUnlock();

  /* Move back into "Rx" mode */
  radio->mode = mode_receiving;
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                               | OpMode_Listen_Off
                               | OpMode_Receiver);
}
