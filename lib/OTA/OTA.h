#ifndef H_OTA
#define H_OTA

#include <functional>
#include "crc.h"
#include "devCRSF.h"

#define OTA4_PACKET_SIZE     8U
#define OTA4_CRC_CALC_LEN    offsetof(OTA_Packet4_s, crcLow)
#define OTA8_PACKET_SIZE     13U
#define OTA8_CRC_CALC_LEN    offsetof(OTA_Packet8_s, crc)

// Packet header types (ota.std.type)
#define PACKET_TYPE_RCDATA  0b00
#define PACKET_TYPE_MSPDATA 0b01
#define PACKET_TYPE_TLM     0b11
#define PACKET_TYPE_SYNC    0b10

// Mask used to XOR the ModelId into the SYNC packet for ModelMatch
#define MODELMATCH_MASK 0x3f

#define KEY8    (uint8_t)0x8
#define KEY16   (uint16_t)0x16
#define KEY32   (uint32_t)0x0032


#define TYPE_WAKE_UP 0x28
#define TYPE_WAKE_UP_RESPONCE 0x29
#define TYPE_SERVICE_TO_SYNC 0x38
#define TYPE_SERVICE_TO_SYNC_RESPONCE 0x39
#define TYPE_GPS_RECVEST 0x48
#define TYPE_GPS_RESPONCE 0x49
#define TYPE_PING_RECVEST 0x58
#define TYPE_PONG_RESPONCE 0x59
#define TYPE_TICK_RECVEST 0x68
#define TYPE_TICK_RESPONCE 0x69
#define TYPE_TO_PING_RECVEST 0x56
#define TYPE_TO_PING_RESPONCE 0x57

typedef struct PackMsg {
    uint8_t type;
    union Data {
        public:
        uint32_t key32; // type 0x28 WAKE UP
        struct WakeUpResponce {
            uint8_t id;
            uint8_t key8;
            uint16_t key16;
        } PACKED wake_up_responce; // type 0x29 WAKE UP RECIVE
        struct ServiceToSync {
            uint8_t id;
            uint8_t key8;
            uint16_t fhssconfig;
        } PACKED service_to_sync; // type 0x38 SERVICE TO SYNC
        struct ServiceToSyncResponce {
            uint8_t id;
            uint8_t key8;
            uint16_t key16;
        } PACKED service_to_sync_responce; // type 0x39 SERVICE TO SYNC RESPONCE
        struct GPSRecvest {
            uint8_t id;
            uint8_t key8;
            uint16_t key16;
        } PACKED gps_recvest; // type 0x48 GPS_RECVEST
        struct GPSResponce{
            uint32_t responce;
        } PACKED gps_responce; // type 0x49 GPS_RESPONCE
        struct ToPingRecvest {
            uint8_t id;
            uint8_t key8;
            uint16_t key16;
        } PACKED to_ping_recvest; // type 0x56 TO_PING_RECVEST
        struct ToPingResponce {
            uint8_t id;
            uint8_t key8;
            uint16_t key16;
        } PACKED to_ping_responce; // type 0x57 TO_PING_RESPONCE
        struct Ping {
            uint32_t free;
        } PACKED ping; // type 0x58 PING_RECVEST
        struct Pong {
            uint32_t free;
        } PACKED pong; // type 0x59 PONG_RESPONCE
        struct TickRecvest {
            uint8_t id;
            uint8_t key8;
            uint16_t key16;
        } PACKED tick_recvest; // type 0x68 TICK_RECVEST

        struct TickResponce {
            uint32_t tick;
        } PACKED tick_responce; // type 0x69 TICK_RESPONCE

    } PACKED data;
}  PACKED Pack_msg;

struct MSP_Package {
            uint8_t packageIndex;
            Pack_msg payload;
            
            
        }PACKED;

typedef struct {
    // The packet type must always be the low two bits of the first byte of the
    // packet to match the same placement in OTA_Packet8_s
    uint8_t type: 2,
            crcHigh: 6;
    union {
        /** PACKET_TYPE_RCDATA **/
        /** PACKET_TYPE_MSP **/
        MSP_Package msp_ul;
        /** PACKET_TYPE_SYNC **/
        /** PACKET_TYPE_TLM **/
         // PACKET_TYPE_TLM
    };
    uint8_t crcLow;
} PACKED OTA_PacketMsp_s;

typedef struct {
    uint8_t fhssIndex;
    uint8_t nonce;
    uint8_t switchEncMode:1,
            newTlmRatio:3,
            rateIndex:4;
    uint8_t UID3;
    uint8_t UID4;
    uint8_t UID5;
} PACKED OTA_Sync_s;

typedef struct {
    uint8_t uplink_RSSI_1:7,
            antenna:1;
    uint8_t uplink_RSSI_2:7,
            modelMatch:1;
    uint8_t lq:7,
            mspConfirm:1;
    int8_t SNR;
} PACKED OTA_LinkStats_s;

typedef struct {
    uint8_t raw[5]; // 4x 10-bit channels, see PackUInt11ToChannels4x10 for encoding
} PACKED OTA_Channels_4x10;

typedef struct {
    // The packet type must always be the low two bits of the first byte of the
    // packet to match the same placement in OTA_Packet8_s
    uint8_t type: 2,
            crcHigh: 6;
    union {
        /** PACKET_TYPE_RCDATA **/
        struct {
            OTA_Channels_4x10 ch;
            uint8_t switches:7,
                    ch4:1;
        } rc;
        struct {
            uint32_t packetNum; // LittleEndian
            uint8_t free[2];
        } PACKED dbg_linkstats;
        /** PACKET_TYPE_MSP **/
        struct {
            uint8_t packageIndex;
            uint8_t payload[ELRS_MSP_BYTES_PER_CALL];
        } msp_ul;
        /** PACKET_TYPE_SYNC **/
        OTA_Sync_s sync;
        /** PACKET_TYPE_TLM **/
        struct {
            uint8_t type:ELRS4_TELEMETRY_SHIFT,
                    packageIndex:(8 - ELRS4_TELEMETRY_SHIFT);
            union {
                struct {
                    OTA_LinkStats_s stats;
                    uint8_t free;
                } PACKED ul_link_stats;
                uint8_t payload[ELRS4_TELEMETRY_BYTES_PER_CALL];
            };
        } tlm_dl; // PACKET_TYPE_TLM
    };
    uint8_t crcLow;
} PACKED OTA_Packet4_s;

typedef struct {
    // Like OTA_Packet4_s **the type is always the low two bits of the first byte**,
    // but they are speficied in each type in the union as the other bits there
    // are not universal
    union {
        /** PACKET_TYPE_RCDATA **/
        struct {
            uint8_t packetType: 2,
                    telemetryStatus: 1,
                    uplinkPower: 3, // PowerLevels_e
                    isHighAux: 1, // true if chHigh are AUX6-9
                    ch4: 1;   // AUX1, included up here so ch0 starts on a byte boundary
            OTA_Channels_4x10 chLow;  // CH0-CH3
            OTA_Channels_4x10 chHigh; // AUX2-5 or AUX6-9
        } PACKED rc;
        struct {
            uint8_t packetType; // actually struct rc's first byte
            uint32_t packetNum; // LittleEndian
            uint8_t free[6];
        } PACKED dbg_linkstats;
        /** PACKET_TYPE_MSP **/
        struct {
            uint8_t packetType: 2,
                    packageIndex: 6;
            uint8_t payload[10];
        } msp_ul;
        /** PACKET_TYPE_SYNC **/
        struct {
            uint8_t packetType; // only low 2 bits
            OTA_Sync_s sync;
            uint8_t free[4];
        } PACKED sync;
        /** PACKET_TYPE_TLM **/
        struct {
            uint8_t packetType: 2,
                    containsLinkStats: 1,
                    packageIndex: 5;
            union {
                struct {
                    OTA_LinkStats_s stats;
                    uint8_t payload[10 - sizeof(OTA_LinkStats_s)];
                } PACKED ul_link_stats; // containsLinkStats == true
                uint8_t payload[10]; // containsLinkStats == false
            };
        } PACKED tlm_dl;
    };
    uint16_t crc;  // crc16 LittleEndian
} PACKED OTA_Packet8_s;

typedef struct {
    union {
        OTA_Packet4_s std;
        OTA_Packet8_s full;
        OTA_PacketMsp_s msp;
    };
} PACKED OTA_Packet_s;

extern bool OtaIsFullRes;
extern volatile uint8_t OtaNonce;
extern uint16_t OtaCrcInitializer;
void OtaUpdateCrcInitFromUid();

enum OtaSwitchMode_e { smWideOr8ch = 0, smHybridOr16ch = 1, sm12ch = 2 };
void OtaUpdateSerializers(OtaSwitchMode_e const mode, uint8_t packetSize);
extern OtaSwitchMode_e OtaSwitchModeCurrent;

// CRC
typedef std::function<bool (OTA_Packet_s * const otaPktPtr)> ValidatePacketCrc_t;
typedef std::function<void (OTA_Packet_s * const otaPktPtr)> GeneratePacketCrc_t;
extern ValidatePacketCrc_t OtaValidatePacketCrc;
extern GeneratePacketCrc_t OtaGeneratePacketCrc;
// Value is implicit leading 1, comment is Koopman formatting (implicit trailing 1) https://users.ece.cmu.edu/~koopman/crc/
#define ELRS_CRC_POLY 0x07 // 0x83
#define ELRS_CRC14_POLY 0x2E57 // 0x372b
#define ELRS_CRC16_POLY 0x3D65 // 0x9eb2

#if defined(TARGET_TX) || defined(UNIT_TEST)
typedef std::function<void (OTA_Packet_s * const otaPktPtr, CRSF const * const crsf, bool TelemetryStatus, uint8_t tlmDenom)> PackChannelData_t;
extern PackChannelData_t OtaPackChannelData;
#if defined(UNIT_TEST)
void OtaSetHybrid8NextSwitchIndex(uint8_t idx);
void OtaSetFullResNextChannelSet(bool next);
#endif
#endif

#if defined(TARGET_RX) || defined(UNIT_TEST)
typedef std::function<bool (OTA_Packet_s const * const otaPktPtr, CRSF * const crsf, uint8_t tlmDenom)> UnpackChannelData_t;
extern UnpackChannelData_t OtaUnpackChannelData;
#endif

#if defined(DEBUG_RCVR_LINKSTATS)
extern uint32_t debugRcvrLinkstatsPacketId;
#endif

#endif // H_OTA
