#include "common.h"
#include "OTA.h"
#include <string>

#define SYNCFHSS 5

#define START_COMMAND 0
#define ARRG_COMMAND 1
#define INVALID_COMMAND -1

#define COMMAND_WAKE_UP "wu"
#define COMMAND_SERVICE_TO_SYNC "ss"
#define COMMAND_GPS_RECVEST "gr"
#define COMMAND_TO_PING_RECVEST "pr"
#define COMMAND_TICK_RECVEST "tr"

namespace TXCommand{


    uint8_t command = INVALID_COMMAND;
    uint8_t line[3] = {0,0,0};
    uint8_t arrg[3] = {0,0,0};
    uint8_t lineiter;
    uint8_t arrgiter;
    uint32_t lastTick;

    void (*loopfunc)() = nullptr;
    
    inline void loop(){
        if (loopfunc != nullptr)
            loopfunc();
    }

    void serviceToSyncloop(){
        if (ssResponce){
            loopfunc = nullptr;
            setupFHSSChannel(SYNCFHSS);
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

    void wakeUp(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_WAKE_UP;
        otaPkt.msp.msp_ul.payload.key32 = KEY32;
        
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