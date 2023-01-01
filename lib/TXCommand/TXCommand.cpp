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
        double aver = 0;
        double aver10 = 0;
        double aver50 = 0;
        double aver100 = 0;
        double aver500 = 0;
        double max;
        double min;

        if (kalman_t.init){
            _time_v.push_back(kalman_t.calc((double)airtime));
            if (airtime > max_airtime)
                max_airtime = airtime;
            if (airtime < min_airtime)
                min_airtime = airtime;
            // if (_time_ar.size() < 500)
            // {
            //     _time_ar.push_back(kalman_aver2d.calc((double)airtime));
            //     ++_time_i;
            // }
            // else{
                if (_time_i >= 1000)
                    _time_i = 0;
                _time_ar[_time_i] = (double)airtime;
                int iter = _time_i;
                ++_time_i;
                
                min = airtime;
                for (int i = 0; i < AR_SIZE; i++)
                {
                    if (i < 10)
                        aver10 += _time_ar[iter];
                    if (i < 50)
                        aver50 += _time_ar[iter];
                    if (i < 100)
                        aver100 += _time_ar[iter];
                    if (i < 500)
                        aver500 += _time_ar[iter];
                    aver += _time_ar[iter];
                    if (max < _time_ar[iter])
                        max = _time_ar[iter];
                    else if (min > _time_ar[iter])
                        min = _time_ar[iter];
                    --iter;
                    if (iter < 0)
                        iter = AR_SIZE - 1;
                }
                aver /= (double)AR_SIZE;
                aver10 /= (double)10;
                aver50 /= (double)50;
                aver100 /= (double)100;
                aver500 /= (double)500;
                // aver = std::reduce(_time_ar.cbegin(), _time_ar.cend()) / (double)_time_ar.size();
                // max = *std::max_element(_time_ar.begin(), _time_ar.end());
                // min = *std::min_element(_time_ar.begin(), _time_ar.end());
                char str[50];
            //int l = sprintf(str, "aver = %lf, raw aver = %lf, min el = %lf, max el = %lf, min air = %lu, max air = %lu\n", _aver, aver, min, max, min_airtime, max_airtime);
            int l = sprintf(str, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", airtime, aver10, aver50, aver100, aver500, aver, min, max);

            Serial.write(str, l);

            // }
            
        }
        else{
            kalman_t.resetTo((double)airtime);
            kalman_aver2d.resetTo((double)airtime);
            kalman_aver3d.resetTo((double)airtime);
            max_airtime = airtime;
            min_airtime = airtime;
        }

        if (_time_v.size() < 100)
            _isResponce = true;
        else {
            aver = std::reduce(_time_v.cbegin(), _time_v.cend()) / (double)_time_v.size();
            max = *std::max_element(_time_v.begin(), _time_v.end());
            min = *std::min_element(_time_v.begin(), _time_v.end());

            _time_v.clear();
            if (_time_v.capacity() < 100)
                _time_v.reserve(100);
            if (kalman_aver.init){
                _aver = kalman_aver.calc(aver);
            }
            else{
                kalman_aver.resetTo(aver);
                // kalman_aver2d.resetTo(aver);
                // kalman_aver3d.resetTo(aver);
                _aver = aver;
            }
            _isResponce = true;
            // char str[50];
            // //int l = sprintf(str, "aver = %lf, raw aver = %lf, min el = %lf, max el = %lf, min air = %lu, max air = %lu\n", _aver, aver, min, max, min_airtime, max_airtime);
            // int l = sprintf(str, "%lf,%lf,%lf,%lf,%lu,%lu\n", _aver, aver, min, max, min_airtime, max_airtime);

            // Serial.write(str, l);
            
        }
        // char str[50];
        // int l = sprintf(str, "aver = %lf, airtime = %lu", _aver, airtime);
        // Serial.write(str, l);
    }
    

    void TXCommand::TickCallBack(uint32_t tick){
        tickrec.TickCallBack(tick);
    }
    

    void TXCommand::TickRecvest::TickCallBack(uint32_t tick){
        
        uint32_t aver = 0;
        uint32_t aver10 = 0;
        uint32_t aver50 = 0;
        uint32_t aver100 = 0;
        uint32_t aver500 = 0;
        uint32_t max;
        uint32_t min;
        if (_time_i >= 1000)
            _time_i = 0;
        _time_ar[_time_i] = tick;
        int iter = _time_i;
        ++_time_i;
                
        min = tick;
        for (int i = 0; i < AR_SIZE; i++)
        {
            if (i < 10)
                aver10 += _time_ar[iter];
            if (i < 50)
                aver50 += _time_ar[iter];
            if (i < 100)
                aver100 += _time_ar[iter];
            if (i < 500)
                aver500 += _time_ar[iter];
            aver += _time_ar[iter];
            if (max < _time_ar[iter])
                max = _time_ar[iter];
            else if (min > _time_ar[iter])
                min = _time_ar[iter];
            --iter;
            if (iter < 0)
                iter = AR_SIZE - 1;
        }
        aver /= AR_SIZE;
        aver10 /= 10;
        aver50 /= 50;
        aver100 /= 100;
        aver500 /= 500;
                // aver = std::reduce(_time_ar.cbegin(), _time_ar.cend()) / (double)_time_ar.size();
                // max = *std::max_element(_time_ar.begin(), _time_ar.end());
                // min = *std::min_element(_time_ar.begin(), _time_ar.end());
        char str[50];
            //int l = sprintf(str, "aver = %lf, raw aver = %lf, min el = %lf, max el = %lf, min air = %lu, max air = %lu\n", _aver, aver, min, max, min_airtime, max_airtime);
            int l = sprintf(str, "%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n", tick, aver10, aver50, aver100, aver500, aver, min, max);

            Serial.write(str, l);

        // if (kalman_t.init){
        //    // _time_v.push_back(kalman_t.calc((double)tick));
        // }
        // else{
        //     kalman_t.resetTo((double)tick);
        // }

        // if (_time_v.size() > 100)
        // {
        //     _time_v.pop_front();
        //     _aver = std::reduce(_time_v.cbegin(), _time_v.cend()) / (double)_time_v.size();
        //     if (kalman_aver.init)
        //         kalman_aver.calc(_aver);
        //     else
        //         kalman_aver.resetTo(_aver);
        // }
        // char str[50];
        //     int l = sprintf(str, "tick = %lu\n", tick);
        //     Serial.write(str, l);
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
        else if (strcmp((char*)line, COMMAND_TICK_RECVEST) == 0){
            isresponce = false;
            toTickRecvest();
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
