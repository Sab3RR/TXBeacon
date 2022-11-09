#include "common.h"
#include "TXCommand.h"
#include "OTA.h"
#include <string>
#include "FHSS.h"





    

     

    void TXCommand::loop(){
        if (loopfunc != nullptr)
            loopfunc();
    }

    void TXCommand::serviceToSyncloop(){
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

    

    void TXCommand::serviceToSync(){
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
        loopfunc = std::bind(&TXCommand::serviceToSyncloop, this);
        
        

    }

    void TXCommand::sendTrashPacket(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
    }

    void TXCommand::debugChannel(){
         setupFHSSChannel(atoi((char*)arrg));
         loopfunc = std::bind(&TXCommand::sendTrashPacket, this);
         
    }

    

    void TXCommand::sendToPingRecvest(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        PackMsg::ToPingRecvest& to_ping_recvest = otaPkt.msp.msp_ul.payload.to_ping_recvest;
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_TO_PING_RECVEST;
        to_ping_recvest.id = atoi((char*)arrg);
        to_ping_recvest.key8 = KEY8;
        to_ping_recvest.key16 = KEY16;
    }

    void TXCommand::toPingRecvest(){
        if (topingrecvest && millis() - lasttopingrecvest > 1000){
            sendToPingRecvest();
        }
        else if(!topingrecvest){

        }
    }
    

    void TXCommand::wakeUp(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_WAKE_UP;
        otaPkt.msp.msp_ul.payload.key32 = KEY32;
        
        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
    }

    void TXCommand::GPSRecvestloop(){
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

    void TXCommand::sendGPSRecvest(){
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

    void TXCommand::proccess(){
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
            loopfunc = std::bind(&TXCommand::GPSRecvestloop, this);
            grResponce = true;
        }
        
        
    }


    void TXCommand::encode (char c){
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
