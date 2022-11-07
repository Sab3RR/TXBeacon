#include "common.h"
#include "TXCommand.h"
#include "OTA.h"
#include <string>
#include "FHSS.h"

#define SYNCFHSS 5

#define START_COMMAND 0
#define ARRG_COMMAND 1
#define INVALID_COMMAND -1

#define COMMAND_WAKE_UP "wu"
#define COMMAND_SERVICE_TO_SYNC "ss"
#define COMMAND_GPS_RECVEST "gr"
#define COMMAND_TO_PING_RECVEST "pr"
#define COMMAND_TICK_RECVEST "tr"
#define COMMAND_DEBUG_CHANNEL "dc"

namespace TXCommand{

    uint8_t command = INVALID_COMMAND;
    uint8_t line[3] = {0,0,0};
    uint8_t arrg[3] = {0,0,0};
    uint8_t lineiter;
    uint8_t arrgiter;
    uint32_t lastTick;
    uint32_t lastgrRecvest;

    void (*loopfunc)() = nullptr;
    
    

    inline void loop(){
        if (loopfunc != nullptr)
            loopfunc();
    }

    static void serviceToSyncloop(){
        if (ssResponce){
            loopfunc = nullptr;
            setupFHSSChannel(SYNCFHSS);
            char str[50];
            int l = sprintf(str, "success to Sync");
            Serial.write(str, l);
            ssResponce = false;
        }
        else if (millis() - lastTick > 1000)
            serviceToSync();
    }

    

    void serviceToSync(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_SERVICE_TO_SYNC;
        otaPkt.msp.msp_ul.payload.service_to_sync.id = atoi((char*)arrg);
        otaPkt.msp.msp_ul.payload.service_to_sync.key8 = KEY8;
        otaPkt.msp.msp_ul.payload.service_to_sync.fhssconfig = SYNCFHSS;
        syncResponceId = otaPkt.msp.msp_ul.payload.service_to_sync.id;

        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
        lastTick = millis();
        loopfunc = serviceToSyncloop;
        

    }

    void sendTrashPacket(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
    }

    void debugChannel(){
         setupFHSSChannel(atoi((char*)arrg));
         loopfunc = sendTrashPacket;
    }
    

    void wakeUp(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_WAKE_UP;
        otaPkt.msp.msp_ul.payload.key32 = KEY32;
        
        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
    }

    void GPSRecvestloop(){
        if (!grResponce && millis() - lastgrRecvest < 1000)
            return;
        if (gpsIter > 5){
            gpsIter = 0;
            grResponce = false;
            loopfunc = nullptr;
            return;
        }

        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_GPS_RECVEST;
        otaPkt.msp.msp_ul.packageIndex = gpsIter;
        otaPkt.msp.msp_ul.payload.gps_recvest.id = atoi((char*)arrg);
        otaPkt.msp.msp_ul.payload.gps_recvest.key8 = KEY8;
        otaPkt.msp.msp_ul.payload.gps_recvest.key16 = KEY16;

        grResponce = false;
        lastgrRecvest = millis();
        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
    }

    void sendGPSRecvest(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_GPS_RECVEST;
        otaPkt.msp.msp_ul.packageIndex = atoi((char*)arrg);
        otaPkt.msp.msp_ul.payload.gps_recvest.id = 2;
        otaPkt.msp.msp_ul.payload.gps_recvest.key8 = KEY8;
        otaPkt.msp.msp_ul.payload.gps_recvest.key16 = KEY16;

        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
    }

    void proccess(){
        if (strcmp((char*)line, COMMAND_WAKE_UP) == 0)
            wakeUp();
        else if (strcmp((char*)line, COMMAND_SERVICE_TO_SYNC) == 0)
        {
            serviceToSync();
        }
        else if (strcmp((char*)line, COMMAND_DEBUG_CHANNEL) == 0){
            debugChannel();
        }
        else if (strcmp((char*)line, COMMAND_GPS_RECVEST) == 0){
            sendGPSRecvest();
        }
        
        
    }


    void encode (char c){
        switch (command){
            case START_COMMAND:
                line[lineiter] = c;
                lineiter++;
                if (lineiter > 1)
                    command = INVALID_COMMAND;
                break;
            case ARRG_COMMAND:
                arrg[arrgiter] = c;
                arrgiter++;
                if (arrgiter > 1)
                    command = INVALID_COMMAND;
                break;
            default:
                command = INVALID_COMMAND;
                
        }

        switch (c){
            case 'x':
                command = START_COMMAND;
                lineiter = 0;
                break;
            case ' ':
                command = ARRG_COMMAND;
                arrgiter = 0;
                break;
            case '\n':
                proccess();

        }
        Serial.write(c);
    }
}