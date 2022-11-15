#include "common.h"
#include "OTA.h"
#include <string>
#include "FHSS.h"
#include <functional>
#include <list>

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

#define __PREALLOC_SIZE__ 100



class TXCommand {
public:
    TXCommand() : loopfunc(nullptr), TXDoneCallBack(nullptr){
        
    }
    bool ssResponce = false;
    int syncResponceId = -1;


    bool grResponce = false;


    bool prResponce = false;
    bool isresponce = false;
    uint32_t lastCall;
    uint8_t gpsIter = 0;
    double lat;
    double lng;
    double alt;

    struct Kalman{
        const double R = 5000;
        const double H = 1.00;
        double Q;
        double P;
        double U_hat;
        double K;

        Kalman() : Q(10), P(0), U_hat(0), K(0){

        }

        void resetTo(double res){
            Q = 10;
            P = 0;
            U_hat = res;
            K = 0;
        }

        double calc(double U){
            K = P * H / (H * P * H + R);
            U_hat = U_hat + K * (U - H * U_hat);
            P = (1 - K * H) * P + Q;

            return U_hat;
        }
    } kalman;
    

    std::list<uint32_t> time_l;

    std::function<void()> loopfunc;
    std::function<void()> TXDoneCallBack;

    bool get_ssResponce(){ return ssResponce;}
    void set_ssResponce(bool s){ ssResponce = s;}
    int get_syncRespId(){return syncResponceId;}
    void set_syncRespId(int s){ syncResponceId = s;}
    bool get_grResp(){return grResponce;}
    void set_grResp(bool s){grResponce = s;}
    inline void inc_gpsIter(){++gpsIter;}
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
    uint32_t lasttopingrecvest;
    uint32_t lasttotickrecvest;
    bool topingrecvest = false;

    void toTickRecvestCallBack();

    void PongCallBack(uint32_t pong);
    void TickCallBack(uint32_t tick);

    void serviceToSyncloop();
    void sendTrashPacket();
    void debugChannel();
    void sendToPingRecvest();
    void sendPing();
    void sendToTickRecvest();
    void toPingRecvest();
    void pingRecvest();
    void toTickRecvest();
    void wakeUp();
    void GPSRecvestloop();
    void sendGPSRecvest();
    void proccess();
};