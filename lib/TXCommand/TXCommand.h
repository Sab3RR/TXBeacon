#include "common.h"
#include "OTA.h"
#include <string>
#include "FHSS.h"
#include <functional>

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

class TXCommand {
public:
    TXCommand() : ssResponce(false), syncResponceId(-1), grResponce(false), gpsIter(0), loopfunc(nullptr){
        
    }
    bool ssResponce;
    int syncResponceId;
    bool grResponce;
    uint8_t gpsIter;
    double lat;
    double lng;
    double alt;

    std::function<void()> loopfunc;

    bool get_ssResponce(){ return ssResponce;}
    void set_ssResponce(bool s){ ssResponce = s;}
    int get_syncRespId(){return syncResponceId;}
    void set_syncRespId(int s){ syncResponceId = s;}
    bool get_grResp(){return grResponce;}
    void set_grResp(bool s){grResponce = s;}
    void inc_gpsIter(){++gpsIter;}
    double get_lat(){ return lat;}
    double get_lng(){ return lng;}
    double get_alt(){ return alt;}
    void set_1lat(uint32_t s){((uint32_t*)&lat)[0] = s;}
    void set_2lat(uint32_t s){((uint32_t*)&lat)[1] = s;}
    void set_1lng(uint32_t s){((uint32_t*)&lng)[0] = s;}
    void set_2lng(uint32_t s){((uint32_t*)&lng)[1] = s;}
    void set_1alt(uint32_t s){((uint32_t*)&alt)[0] = s;}
    void set_2alt(uint32_t s){((uint32_t*)&alt)[1] = s;}

    static void setupFHSSChannel(const uint8_t channel)
    {
        Radio.SetFrequencyReg(FHSSgetCurrFreq(channel));
    }
    void serviceToSync();
    void loop();
    void encode (char c);

private:
    uint8_t command = INVALID_COMMAND;
    uint8_t line[3] = {0,0,0};
    uint8_t arrg[3] = {0,0,0};
    uint8_t lineiter;
    uint8_t arrgiter;
    uint32_t lastTick;
    uint32_t lastgrRecvest;

    void serviceToSyncloop();
    void sendTrashPacket();
    void debugChannel();
    void wakeUp();
    void GPSRecvestloop();
    void sendGPSRecvest();
    void proccess();
};