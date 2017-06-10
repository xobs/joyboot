
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "kl17.h"
#include "radio.h"
#include "TransceiverReg.h"
#include "spi.h"

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
  RADIO_DataModul, DataModul_DataMode_Packet | DataModul_Modulation_Fsk | DataModul_ModulationShaping_NoShaping,

  /* Radio bit rate initialization @0x03-0x04*/
  RADIO_BitrateMsb, BitrateMsb_55555,
  RADIO_BitrateLsb, BitrateLsb_55555,

  /* Radio frequency deviation initialization @0x05-0x06*/
  RADIO_FdevMsb, FdevMsb_50000,
  RADIO_FdevLsb, FdevLsb_50000,

  /* Disable AES encryption -- the key keeps its values across resets */
  RADIO_AesKey1, 0,
  RADIO_AesKey2, 0,
  RADIO_AesKey3, 0,
  RADIO_AesKey4, 0,
  RADIO_AesKey5, 0,
  RADIO_AesKey6, 0,
  RADIO_AesKey7, 0,
  RADIO_AesKey8, 0,
  RADIO_AesKey9, 0,
  RADIO_AesKey10, 0,
  RADIO_AesKey11, 0,
  RADIO_AesKey12, 0,
  RADIO_AesKey13, 0,
  RADIO_AesKey14, 0,
  RADIO_AesKey15, 0,
  RADIO_AesKey16, 0,

  /* Radio RF frequency initialization @0x07-0x09*/
  /*Default Frequencies*/
#define DEFAULT_FRF_433

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

#ifdef DEFAULT_FRF_433
  RADIO_FrfMsb, FrfMsb_433,
  RADIO_FrfMid, FrfMid_433,
  RADIO_FrfLsb, FrfLsb_433,
#endif

#ifdef DEFAULT_FRF_920                                                          //JAPAN
  RADIO_FrfMsb, FrfMsb_920,
  RADIO_FrfMid, FrfMid_920,
  RADIO_FrfLsb, FrfLsb_920,
#endif

  /* Radio RegAfcCtrl initialization @0x0B*/
  //RADIO_AfcCtrl, AfcCtrl_AfcLowBeta_Off,

  /* Radio output power initialization @0x11*/
  RADIO_PaLevel, PaLevel_Pa0_On | PaLevel_Pa1_Off | PaLevel_Pa2_Off | 0x1F,
  //RADIO_PaLevel, 0x7F,

  /* Radio Rise/Fall time of ramp up/down in FSK initialization @0x12*/
  RADIO_PaRamp, PaRamp_500,

  /* Radio overload current protection for PA initialization 0x13*/
  //RADIO_Ocp, Ocp_Ocp_On | 0x0C,
  RADIO_Ocp, Ocp_Ocp_Off,

  /* Radio LNA gain and input impedance initialization @0x18*/
  RADIO_Lna, Lna_LnaZin_50 | Lna_LnaGain_Agc,
  //RADIO_Lna, Lna_LnaZin_50 | 0x08,
  //RADIO_Lna, Lna_LnaZin_200 | 0x08,

  /* Radio channel filter bandwidth initialization @0x19*/
  RADIO_RxBw, DccFreq_2 | RxBw_250000,

  /* Radio channel filter bandwidth for AFC operation initialization @0x1A*/
  RADIO_AfcBw, DccFreq_2 | RxBw_250000,

  /* Radio automatic frequency control initialization @0x1E*/
  //RADIO_AfcFei, AfcFei_AfcAuto_Off | AfcFei_AfcAutoClear_On,

  /* Radio Rssi threshold initialization @0x29*/
  // RSSIthres = [-174 + NF +10*log(2*RxBw) + DemodSNR] dBm
  // NF = 7dB
  // DemodSnr = 8dB
  // RxBw depends on frequency bands and profiles
  //RADIO_RssiThresh, 0xe4, // -101 dBm for 333.3 Khz singleside channel filter bandwith
  RADIO_RssiThresh, 0xdc, // -101 dBm for 333.3 Khz singleside channel filter bandwith

  /* Radio RegTimeoutRxStart initialization @0x2A*/
  /* Radio RegTimeoutRssiThresh initialization @0x2B*/
  RADIO_RxTimeout1, 0x00, //disable timeout rx start
  RADIO_RxTimeout2, 0x00, //disable timeout rx start

  /* MKW01 preamble size initialization @0x2C-0x2D*/
  RADIO_PreambleMsb, 0x00,
  RADIO_PreambleLsb, 0x10,

  /* Radio sync word control and value initialization @0x2E-0x30*/
  RADIO_SyncConfig, SyncConfig_Sync_On | SyncConfig_FifioFill_ifSyncAddres | SyncConfig_SyncSize_2,
  RADIO_SyncValue1, 0x2d, //SFD value for uncoded with phySUNMRFSKSFD = 0
  RADIO_SyncValue2, 0x55, //SFD value for uncoded with phySUNMRFSKSFD = 0

  /* Radio packet mode config */
  RADIO_PacketConfig1, PacketConfig1_PacketFormat_Variable_Length /*| PacketConfig1_AddresFiltering_Node_Or_Broadcast*/ | PacketConfig1_Crc_On | PacketConfig1_DcFree_Whitening,
  RADIO_PacketConfig2, PacketConfig2_AutoRxRestart_On | PacketConfig2_Aes_Off | 0x10,

  /* Radio payload length initialization */
  RADIO_PayloadLength, RADIO_FIFO_DEPTH,  //max length in rx

  RADIO_DioMapping1, DIO0_RxCrkOk | DIO1_TxFifoNotEmpty,
  RADIO_DioMapping2, 0x07, // turn off clock output

  /* Fading margin improvement, recommended by RFM69HW manual */
  RADIO_TestDagc, 0x30,

  /* Prep a temperature sample */
  RADIO_Temp1, REG_TEMP1_START,
};

// These are indexed by the values of ModemConfigChoice
// Stored in flash (program) memory to save SRAM
// It is important to keep the modulation index for FSK between 0.5 and 10
// modulation index = 2 * Fdev / BR
// Note that I have not had much success with FSK with Fd > ~5
// You have to construct these by hand, using the data from the RF69 Datasheet :-(
// or use the SX1231 starter kit software (Ctl-Alt-N to use that without a connected radio)
// RH_RF69_REG_02_DATAMODUL
#define RH_RF69_DATAMODUL_DATAMODE                          0x60
#define RH_RF69_DATAMODUL_DATAMODE_PACKET                   0x00
#define RH_RF69_DATAMODUL_DATAMODE_CONT_WITH_SYNC           0x40
#define RH_RF69_DATAMODUL_DATAMODE_CONT_WITHOUT_SYNC        0x60
#define RH_RF69_DATAMODUL_MODULATIONTYPE                    0x18
#define RH_RF69_DATAMODUL_MODULATIONTYPE_FSK                0x00
#define RH_RF69_DATAMODUL_MODULATIONTYPE_OOK                0x08
#define RH_RF69_DATAMODUL_MODULATIONSHAPING                 0x03
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE        0x00
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0       0x01
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_5       0x02
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT0_3       0x03
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE        0x00
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_BR          0x01
#define RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_2BR         0x02
#define CONFIG_FSK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_NONE)
#define CONFIG_GFSK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_FSK | RH_RF69_DATAMODUL_MODULATIONSHAPING_FSK_BT1_0)
#define CONFIG_OOK (RH_RF69_DATAMODUL_DATAMODE_PACKET | RH_RF69_DATAMODUL_MODULATIONTYPE_OOK | RH_RF69_DATAMODUL_MODULATIONSHAPING_OOK_NONE)

// Choices for RH_RF69_REG_37_PACKETCONFIG1:
// RH_RF69_REG_37_PACKETCONFIG1
#define RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE         0x80
#define RH_RF69_PACKETCONFIG1_DCFREE                        0x60
#define RH_RF69_PACKETCONFIG1_DCFREE_NONE                   0x00
#define RH_RF69_PACKETCONFIG1_DCFREE_MANCHESTER             0x20
#define RH_RF69_PACKETCONFIG1_DCFREE_WHITENING              0x40
#define RH_RF69_PACKETCONFIG1_DCFREE_RESERVED               0x60
#define RH_RF69_PACKETCONFIG1_CRC_ON                        0x10
#define RH_RF69_PACKETCONFIG1_CRCAUTOCLEAROFF               0x08
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING              0x06
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NONE         0x00
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE         0x02
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC      0x04
#define RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_RESERVED 0x06
#define CONFIG_NOWHITE (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_NONE |  RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC)
#define CONFIG_WHITE (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_WHITENING |  RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC)
#define CONFIG_MANCHESTER (RH_RF69_PACKETCONFIG1_PACKETFORMAT_VARIABLE | RH_RF69_PACKETCONFIG1_DCFREE_MANCHESTER |  RH_RF69_PACKETCONFIG1_CRC_ON | RH_RF69_PACKETCONFIG1_ADDRESSFILTERING_NODE_BC)

struct modem_config
{
	uint8_t    reg_02;   ///< Value for register RH_RF69_REG_02_DATAMODUL
	uint8_t    reg_03;   ///< Value for register RH_RF69_REG_03_BITRATEMSB
	uint8_t    reg_04;   ///< Value for register RH_RF69_REG_04_BITRATELSB
	uint8_t    reg_05;   ///< Value for register RH_RF69_REG_05_FDEVMSB
	uint8_t    reg_06;   ///< Value for register RH_RF69_REG_06_FDEVLSB
	uint8_t    reg_19;   ///< Value for register RH_RF69_REG_19_RXBW
	uint8_t    reg_1a;   ///< Value for register RH_RF69_REG_1A_AFCBW
	uint8_t    reg_37;   ///< Value for register RH_RF69_REG_37_PACKETCONFIG1
};


static const struct modem_config MODEM_CONFIG_TABLE[] =
{
    //  02,        03,   04,   05,   06,   19,   1a,  37
    // FSK, No Manchester, no shaping, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate
    // Low modulation indexes of ~ 1 at slow speeds do not seem to work very well. Choose MI of 2.
    { CONFIG_FSK,  0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2Fd5
    { CONFIG_FSK,  0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb2_4Fd4_8
    { CONFIG_FSK,  0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb4_8Fd9_6

    { CONFIG_FSK,  0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE}, // FSK_Rb9_6Fd19_2
    { CONFIG_FSK,  0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE}, // FSK_Rb19_2Fd38_4
    { CONFIG_FSK,  0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE}, // FSK_Rb38_4Fd76_8

    { CONFIG_FSK,  0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE}, // FSK_Rb57_6Fd120
    { CONFIG_FSK,  0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE}, // FSK_Rb125Fd125
    { CONFIG_FSK,  0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE}, // FSK_Rb250Fd250
    { CONFIG_FSK,  0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE}, // FSK_Rb55555Fd50

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // GFSK (BT=1.0), No Manchester, whitening, CRC, no address filtering
    // AFC BW == RX BW == 2 x bit rate
    { CONFIG_GFSK, 0x3e, 0x80, 0x00, 0x52, 0xf4, 0xf5, CONFIG_WHITE}, // GFSK_Rb2Fd5
    { CONFIG_GFSK, 0x34, 0x15, 0x00, 0x4f, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb2_4Fd4_8
    { CONFIG_GFSK, 0x1a, 0x0b, 0x00, 0x9d, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb4_8Fd9_6

    { CONFIG_GFSK, 0x0d, 0x05, 0x01, 0x3b, 0xf4, 0xf4, CONFIG_WHITE}, // GFSK_Rb9_6Fd19_2
    { CONFIG_GFSK, 0x06, 0x83, 0x02, 0x75, 0xf3, 0xf3, CONFIG_WHITE}, // GFSK_Rb19_2Fd38_4
    { CONFIG_GFSK, 0x03, 0x41, 0x04, 0xea, 0xf2, 0xf2, CONFIG_WHITE}, // GFSK_Rb38_4Fd76_8

    { CONFIG_GFSK, 0x02, 0x2c, 0x07, 0xae, 0xe2, 0xe2, CONFIG_WHITE}, // GFSK_Rb57_6Fd120
    { CONFIG_GFSK, 0x01, 0x00, 0x08, 0x00, 0xe1, 0xe1, CONFIG_WHITE}, // GFSK_Rb125Fd125
    { CONFIG_GFSK, 0x00, 0x80, 0x10, 0x00, 0xe0, 0xe0, CONFIG_WHITE}, // GFSK_Rb250Fd250
    { CONFIG_GFSK, 0x02, 0x40, 0x03, 0x33, 0x42, 0x42, CONFIG_WHITE}, // GFSK_Rb55555Fd50

    //  02,        03,   04,   05,   06,   19,   1a,  37
    // OOK, No Manchester, no shaping, whitening, CRC, no address filtering
    // with the help of the SX1231 configuration program
    // AFC BW == RX BW
    // All OOK configs have the default:
    // Threshold Type: Peak
    // Peak Threshold Step: 0.5dB
    // Peak threshiold dec: ONce per chip
    // Fixed threshold: 6dB
    { CONFIG_OOK,  0x7d, 0x00, 0x00, 0x10, 0x88, 0x88, CONFIG_WHITE}, // OOK_Rb1Bw1
    { CONFIG_OOK,  0x68, 0x2b, 0x00, 0x10, 0xf1, 0xf1, CONFIG_WHITE}, // OOK_Rb1_2Bw75
    { CONFIG_OOK,  0x34, 0x15, 0x00, 0x10, 0xf5, 0xf5, CONFIG_WHITE}, // OOK_Rb2_4Bw4_8
    { CONFIG_OOK,  0x1a, 0x0b, 0x00, 0x10, 0xf4, 0xf4, CONFIG_WHITE}, // OOK_Rb4_8Bw9_6
    { CONFIG_OOK,  0x0d, 0x05, 0x00, 0x10, 0xf3, 0xf3, CONFIG_WHITE}, // OOK_Rb9_6Bw19_2
    { CONFIG_OOK,  0x06, 0x83, 0x00, 0x10, 0xf2, 0xf2, CONFIG_WHITE}, // OOK_Rb19_2Bw38_4
    { CONFIG_OOK,  0x03, 0xe8, 0x00, 0x10, 0xe2, 0xe2, CONFIG_WHITE}, // OOK_Rb32Bw64

//    { CONFIG_FSK,  0x68, 0x2b, 0x00, 0x52, 0x55, 0x55, CONFIG_WHITE}, // works: Rb1200 Fd 5000 bw10000, DCC 400
//    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x52, 0x52, CONFIG_WHITE}, // works 10/40/80
//    { CONFIG_FSK,  0x0c, 0x80, 0x02, 0x8f, 0x53, 0x53, CONFIG_WHITE}, // works 10/40/40

};

    typedef enum
    {
	FSK_Rb2Fd5 = 0,	    ///< FSK, Whitening, Rb = 2kbs,    Fd = 5kHz
	FSK_Rb2_4Fd4_8,     ///< FSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz
	FSK_Rb4_8Fd9_6,     ///< FSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz
	FSK_Rb9_6Fd19_2,    ///< FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
	FSK_Rb19_2Fd38_4,   ///< FSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
	FSK_Rb38_4Fd76_8,   ///< FSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
	FSK_Rb57_6Fd120,    ///< FSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	FSK_Rb125Fd125,     ///< FSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	FSK_Rb250Fd250,     ///< FSK, Whitening, Rb = 250kbs,  Fd = 250kHz
	FSK_Rb55555Fd50,    ///< FSK, Whitening, Rb = 55555kbs,Fd = 50kHz for RFM69 lib compatibility

	GFSK_Rb2Fd5,	      ///< GFSK, Whitening, Rb = 2kbs,    Fd = 5kHz
	GFSK_Rb2_4Fd4_8,    ///< GFSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz
	GFSK_Rb4_8Fd9_6,    ///< GFSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz
	GFSK_Rb9_6Fd19_2,   ///< GFSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
	GFSK_Rb19_2Fd38_4,  ///< GFSK, Whitening, Rb = 19.2kbs, Fd = 38.4kHz
	GFSK_Rb38_4Fd76_8,  ///< GFSK, Whitening, Rb = 38.4kbs, Fd = 76.8kHz
	GFSK_Rb57_6Fd120,   ///< GFSK, Whitening, Rb = 57.6kbs, Fd = 120kHz
	GFSK_Rb125Fd125,    ///< GFSK, Whitening, Rb = 125kbs,  Fd = 125kHz
	GFSK_Rb250Fd250,    ///< GFSK, Whitening, Rb = 250kbs,  Fd = 250kHz
	GFSK_Rb55555Fd50,   ///< GFSK, Whitening, Rb = 55555kbs,Fd = 50kHz

	OOK_Rb1Bw1,         ///< OOK, Whitening, Rb = 1kbs,    Rx Bandwidth = 1kHz.
	OOK_Rb1_2Bw75,      ///< OOK, Whitening, Rb = 1.2kbs,  Rx Bandwidth = 75kHz.
	OOK_Rb2_4Bw4_8,     ///< OOK, Whitening, Rb = 2.4kbs,  Rx Bandwidth = 4.8kHz.
	OOK_Rb4_8Bw9_6,     ///< OOK, Whitening, Rb = 4.8kbs,  Rx Bandwidth = 9.6kHz.
	OOK_Rb9_6Bw19_2,    ///< OOK, Whitening, Rb = 9.6kbs,  Rx Bandwidth = 19.2kHz.
	OOK_Rb19_2Bw38_4,   ///< OOK, Whitening, Rb = 19.2kbs, Rx Bandwidth = 38.4kHz.
	OOK_Rb32Bw64,       ///< OOK, Whitening, Rb = 32kbs,   Rx Bandwidth = 64kHz.

//	Test,
} ModemConfigChoice;

static void spiSend(void *ignored, int count, const void *data) {
  (void)ignored;
  int i;
  const uint8_t *bytes = data;

  for (i = 0; i < count; i++)
    spiTransceive(bytes[i]);
}

static void spiReceive(void *ignored, int count, void *data) {
  (void)ignored;

  int i;
  uint8_t *bytes = data;

  for (i = 0; i < count; i++)
    bytes[i] = spiTransceive(0xff);
}

static void radio_select(KRadioDevice *radio) {
  (void)radio;
  spiAssertCs();
}

static void radio_unselect(KRadioDevice *radio) {
  (void)radio;
  spiDeassertCs();
}

int radioDump(KRadioDevice *radio, uint8_t addr, void *bfr, int count) {

  radio_select(radio);
  spiSend(NULL, 1, &addr);
  spiReceive(NULL, count, bfr);
  radio_unselect(radio);

  return 0;
}

uint8_t data_backing[64];
__attribute__((used))
uint8_t *radioDumpFifo(void) {
  radioDump(NULL, 0, data_backing, sizeof(data_backing));
  return data_backing;
}

__attribute__((used))
uint8_t *radioDumpData(uint8_t start, uint8_t len) {
  radioDump(NULL, start, data_backing, len);
  return data_backing;
}

static void radio_set(KRadioDevice *radio, uint8_t addr, uint8_t val) {

  uint8_t buf[2] = {addr | 0x80, val};

  radio_select(radio);
  spiSend(NULL, 2, buf);
  radio_unselect(radio);
}

static uint8_t radio_get(KRadioDevice *radio, uint8_t addr) {

  uint8_t val;
  radioDump(radio, addr, &val, 1);
  return val;
}
#if 0

void radioPhySetBitRate(KRadioDevice *radio, uint32_t rate) {

  rate = RADIO_XTAL_FREQUENCY / rate;
  radio_set(radio, RADIO_BitrateMsb, rate >> 8);
  radio_set(radio, RADIO_BitrateLsb, rate);
}

static void radio_phy_update_modulation_parameters(KRadioDevice *radio) {

  /* Radio frequency deviation initialization @0x05-0x06*/
  radio_set(radio, RADIO_FdevMsb, Fdev_170000 >> 8);
  radio_set(radio, RADIO_FdevLsb, Fdev_170000 & 0xff);

 /* Radio channel filter bandwidth initialization @0x19*/
  radio_set(radio, RADIO_RxBw, DccFreq_2 | RxBw_250000);

  /* Radio channel filter bandwidth for AFC operation initialization @0x1A*/
  radio_set(radio, RADIO_AfcBw, DccFreq_2 | RxBw_250000);
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

static void radio_phy_force_idle(KRadioDevice *radio) {
  //Put transceiver in Stand-By mode
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                            | OpMode_Listen_Off
                            | OpMode_StandBy);

  //clear the transceiver FIFO
  while(radio_get(radio, RADIO_IrqFlags2) & 0x40)
    (void)radio_get(radio, RADIO_Fifo);
}

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
#endif

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

  radioUnloadPacket(radio);
}

void radioStop(KRadioDevice *radio) {
  radio_set(radio, RADIO_OpMode, 0x80); // force into sleep mode immediately
}

static void radio_set_config(KRadioDevice *radio, unsigned int config_num) {
  if (config_num > ARRAY_SIZE(MODEM_CONFIG_TABLE))
    return;

  const struct modem_config *config = &MODEM_CONFIG_TABLE[config_num];
  radio_set(radio, 0x02, config->reg_02);
  radio_set(radio, 0x03, config->reg_03);
  radio_set(radio, 0x04, config->reg_04);
  radio_set(radio, 0x05, config->reg_05);
  radio_set(radio, 0x06, config->reg_06);
  radio_set(radio, 0x19, config->reg_19);
  radio_set(radio, 0x1a, config->reg_1a);
  radio_set(radio, 0x37, config->reg_37);
}

void radioStart(KRadioDevice *radio) {

  unsigned int reg;

  reg = 0;
  while (reg < ARRAY_SIZE(default_registers)) {
    uint8_t cmd = default_registers[reg++];
    uint8_t dat = default_registers[reg++];

    radio_set(radio, cmd, dat);
  }

  radio_set_config(radio, FSK_Rb2Fd5);

  //radio_phy_update_modulation_parameters(radio);
  //radioPhySetBitRate(radio, 50000);
  //radioPhySetRfDeviation(radio, 26370);
  //radioPhySetRfDeviation(radio, 40625/2);
  //radioPhySetBitRate(radio, 40625);

  radio->channel = 0;
  //radioPhyUpdateRfFrequency(radio, 433923000);

  //radio_set_preamble_length(radio, 3);

  //radio_set_output_power_dbm(radio, 13); /* Max output with PA0 is 13 dBm */

//  radio_set_encoding(radio, encoding_whitening);
//  radio_set_modulation(radio, modulation_fsk_gaussian_bt_0p5);
  //radio_set_encoding(radio, encoding_none);
  //radio_set_modulation(radio, modulation_fsk_gaussian_bt_1p0);
  //radio_set_packet_mode(radio);
  radio_set_broadcast_address(radio, 255);
  radio_set_node_address(radio, 1);

  /* Drain the Fifo */
  while (radio_get(radio, RADIO_IrqFlags2) & IrqFlags2_FifoNotEmpty)
    (void)radio_get(radio, RADIO_Fifo);

  radio_set(radio, RADIO_TestLna, 0x2D); // put LNA into high sensitivity mode

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

int radioTemperature(KRadioDevice *radio) {

  uint8_t buf[2];

  radio_set(radio, REG_TEMP1, REG_TEMP1_START);

  do {
    buf[0] = REG_TEMP1;

    radio_select(radio);
    spiSend(NULL, 1, buf);
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

  /* The length byte is not included in the length calculation. */
  pkt.length = bytes + sizeof(pkt) - 1;
  pkt.src = radio->address;
  pkt.dst = addr;
  pkt.prot = prot;

  /* Ideally, we'd poll for DIO1 to see when the FIFO can accept data.
   * This is not wired up on Orchard, so we can't transmit packets larger
   * than the FIFO.
   */
//  osalDbgAssert(pkt.length < RADIO_FIFO_DEPTH, "Packet is too large");

  //radio_set(radio, RADIO_DioMapping1, DIO0_RxCrkOk | DIO1_TxFifoNotEmpty);
  radio->mode = mode_standby;
  /* Go into standby mode, to prevent the radio from receiving while we're loading the fifo */
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                               | OpMode_Listen_Off
                               | OpMode_StandBy);

  /* Transmit the packet as soon as the first byte enters the FIFO */
  radio_set(radio, RADIO_FifoThresh, 0x80 | pkt.length);

  radio_select(radio);
  reg = RADIO_Fifo | 0x80;  spiSend(NULL, 1, &reg);  /* Select the FIFO */
  spiSend(NULL, sizeof(pkt), &pkt);  /* Load the header into the Fifo */
  spiSend(NULL, bytes, payload);  /* Load the payload into the Fifo */
  radio_unselect(radio);

#if 0
  // set high power modes
  radio_set(radio, RADIO_PaLevel, 0x7F);
  radio_set(radio, RADIO_Ocp, 0x0F);
  radio_set(radio, RADIO_RegTestPa1, 0x5D);
  radio_set(radio, RADIO_RegTestPa2, 0x7C);
#endif

  radio->mode = mode_transmitting;
  /* Enter transmission mode, to actually send packets */
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                               | OpMode_Listen_Off
                               | OpMode_Transmitter);

  /* Wait for the radio to indicate the packet has been sent */
  while(!(radio_get(radio, RADIO_IrqFlags2) & IrqFlags2_PacketSent))
    ;

#if 0
  // turn off high power PA settings to prevent Rx damage (done in either boost or regular case)
  radio_set(radio, RADIO_PaLevel, 0x9F);
  radio_set(radio, RADIO_Ocp, 0x1C);
  radio_set(radio, RADIO_RegTestPa1, 0x55);
  radio_set(radio, RADIO_RegTestPa2, 0x70);
#endif

  /* Move back into "Rx" mode */
  radio->mode = mode_receiving;
  radio_set(radio, RADIO_OpMode, OpMode_Sequencer_On
                               | OpMode_Listen_Off
                               | OpMode_Receiver);
}
