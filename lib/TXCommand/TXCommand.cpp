#include "common.h"
#include "TXCommand.h"
#include "OTA.h"
#include <string>
#include "FHSS.h"
#include <numeric>





    

     

    void TXCommand::loop(){
        if (loopfunc != nullptr)
            loopfunc();
    }

    void TXCommand::PongCallBack(uint32_t pong){
        pingrec.PongCallBack(pong);
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
        otaPkt.msp.msp_ul.payload.data.service_to_sync.id = atoi((char*)arrg);
        otaPkt.msp.msp_ul.payload.data.service_to_sync.key8 = KEY8;
        otaPkt.msp.msp_ul.payload.data.service_to_sync.fhssconfig = SYNCFHSS;
        syncResponceId = otaPkt.msp.msp_ul.payload.data.service_to_sync.id;

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
        PackMsg::Data::ToPingRecvest& to_ping_recvest = otaPkt.msp.msp_ul.payload.data.to_ping_recvest;
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_TO_PING_RECVEST;
        to_ping_recvest.id = atoi((char*)arrg);
        to_ping_recvest.key8 = KEY8;
        to_ping_recvest.key16 = KEY16;

        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
        topingrecvest = false;        
    }

    void TXCommand::toPingResponce(){
        topingrecvest = false;
        // TXDoneCallBack = std::bind(&PingRecvest::TXCallBack, &pingrec);
        pingrec._isResponce = true;
    }

    void TXCommand::toPingRecvest(){
        if (topingrecvest && millis() - lasttopingrecvest > 1000){
            sendToPingRecvest();
        }
        else if(!topingrecvest){
            sendPing();
        }
    }

    // void TXCommand::PingRecvest::TXCallBack(){
    //     _isResponce = true;
    // }

    void ICACHE_RAM_ATTR TXCommand::sendPing(){
        if (!pingrec._isResponce && ESP.getCycleCount() - pingrec.lastCall  < 10000000){
            return;
        }
        
        // char str[50];
        // int l = sprintf(str, "send ping = %lu\n", micros64());
        // Serial.write(str, l);

        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};

        Pack_msg::Data::Ping& ping = otaPkt.msp.msp_ul.payload.data.ping;
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_PING_RECVEST;
        pingrec._isResponce = false;

        OtaGeneratePacketCrc(&otaPkt);
        pingrec.lastCall = ESP.getCycleCount();

        
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
        
    }

    void TXCommand::toTickRecvestCallBack(){
        setupFHSSChannel(20 + atoi((char*)arrg));
        TXDoneCallBack = nullptr;
    }

    void TXCommand::sendToTickRecvest(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        
        Pack_msg::Data::TickRecvest& tick_recvest = otaPkt.msp.msp_ul.payload.data.tick_recvest;
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_TICK_RECVEST;
        tick_recvest.id = atoi((char*)arrg);
        tick_recvest.key8 = KEY8;
        tick_recvest.key16 = KEY16;

        TXDoneCallBack = std::bind(&TXCommand::toTickRecvestCallBack, this);
        OtaGeneratePacketCrc(&otaPkt);
        Radio.TXnb((uint8_t*)&otaPkt, ExpressLRS_currAirRate_Modparams->PayloadLength);
        lasttotickrecvest = millis();
    }
    
    void TXCommand::PingRecvest::PongCallBack(uint32_t pong){
        uint32_t airtime = pong - lastCall;
        double aver;

        if (kalman_t.init){
            _time_v.push_back(kalman_t.calc((double)airtime));
        }
        else{
            kalman_t.resetTo((double)airtime);
        }

        if (_time_v.size() < 100)
            _isResponce = true;
        else {
            aver = std::reduce(_time_v.cbegin(), _time_v.cend()) / (double)_time_v.size();
            _time_v.clear();
            if (_time_v.capacity() < 100)
                _time_v.reserve(100);
            if (kalman_aver.init){
                _aver = kalman_aver3d.calc(kalman_aver2d.calc(kalman_aver.calc(aver)));
            }
            else{
                kalman_aver.resetTo(aver);
                kalman_aver2d.resetTo(aver);
                kalman_aver3d.resetTo(aver);
                _aver = aver;
            }
            _isResponce = true;
            char str[50];
            int l = sprintf(str, "aver = %lf\n", _aver);
            Serial.write(str, l);
            
        }
        // char str[50];
        // int l = sprintf(str, "aver = %lf, airtime = %lu", _aver, airtime);
        // Serial.write(str, l);
    }
    

    inline void TXCommand::TickCallBack(uint32_t tick){
        tickrec.TickCallBack(tick);
    }
    

    void TXCommand::TickRecvest::TickCallBack(uint32_t tick){
        if (kalman_t.init){
            _time_v.push_back(kalman_t.calc((double)tick));
        }
        else{
            kalman_t.resetTo((double)tick);
        }

        if (_time_v.size() > 100)
        {
            _time_v.erase(_time_v.begin());
            _aver = std::reduce(_time_v.cbegin(), _time_v.cend()) / (double)_time_v.size();
            if (kalman_aver.init)
                kalman_aver.calc(_aver);
            else
                kalman_aver.resetTo(_aver);
        }
    }

    void TXCommand::toTickRecvest(){
        if (!isresponce && millis() - lasttotickrecvest > 1000){
            setupFHSSChannel(SYNCFHSS);
            sendToTickRecvest();
        }
        

    }

    void TXCommand::TXPingDoneCallBack(){

    }

    void TXCommand::wakeUp(){
        WORD_ALIGNED_ATTR OTA_Packet_s otaPkt = {0};
        otaPkt.msp.type = PACKET_TYPE_MSPDATA;
        otaPkt.msp.msp_ul.payload.type = TYPE_WAKE_UP;
        otaPkt.msp.msp_ul.payload.data.key32 = KEY32;
        
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
        otaPkt.msp.msp_ul.payload.data.gps_recvest.id = atoi((char*)arrg);
        otaPkt.msp.msp_ul.payload.data.gps_recvest.key8 = KEY8;
        otaPkt.msp.msp_ul.payload.data.gps_recvest.key16 = KEY16;

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
        otaPkt.msp.msp_ul.payload.data.gps_recvest.id = 2;
        otaPkt.msp.msp_ul.payload.data.gps_recvest.key8 = KEY8;
        otaPkt.msp.msp_ul.payload.data.gps_recvest.key16 = KEY16;

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
        else if (strcmp((char*)line, COMMAND_TO_PING_RECVEST) == 0){
            loopfunc = std::bind(&TXCommand::toPingRecvest, this);
            topingrecvest = true;
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
