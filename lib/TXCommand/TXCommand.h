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

template <typename T>
struct Kalman{
    const T R = 1000;
    const double H = 1;
    T Q;
    T P;
    T U_hat;
    T K;

    bool init = false;

    Kalman() : Q(100), P(0), U_hat(0), K(0){

    }

    void resetTo(double res){
        Q = 100;
        P = 0;
        U_hat = res;
        K = 0;
        init = true;
    }

    double calc(double U){
        K = P * H / (H * P * H + R);
        U_hat = U_hat + K * (U - H * U_hat);
        P = (1 - K * H) * P + Q;

        return U_hat;
    }
};

template <typename T>
class Kalman2 {
    double _dNoise; //дисперсия шума
    //double dSignal; //дисперсия сигнала
    double _r; // коэвициент корреляции
    double _en; // дисперсия
    double _Pe;
    double _xx;

    

public:

    bool init = false;
    Kalman2(double dNoise, double r, double en) : _dNoise(dNoise), _r(r), _en(en){

    }
    bool ICACHE_RAM_ATTR getInit() { return init;};

    void ICACHE_RAM_ATTR resetTo(T res){
        _Pe = _dNoise;
        _xx = res;
        if (init) 
            return;
        else
            init = true;
    }

    T ICACHE_RAM_ATTR calc(double z){
        double Pe = _r * _r * _Pe + _en * _en;
        _Pe = (Pe * _dNoise) / (Pe + _dNoise);
        _xx = _r * _xx + _Pe / _dNoise * (z - _r * _xx);
        return (T)_xx;
    }

};


class TXCommand {
public:
    TXCommand() : loopfunc(nullptr), TXDoneCallBack(nullptr), pingrec(*this), tickrec(*this){
        
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
    void PongCallBack(uint32_t pong);
    void TickCallBack(uint32_t tick);
    void toPingResponce();


private:


    class PingRecvest {
        uint32_t lastCall;
        TXCommand& _baseTX;
        bool _isResponce;
        std::vector<double> _time_v;
        std::vector<double> _3calibration;
        std::vector<double> _aver_v;
        double _aver;
        Kalman2<double> kalman_t;
        Kalman2<double> kalman_aver;
        Kalman2<double> kalman_aver2d;
        Kalman2<double> kalman_aver3d;

    public:
        friend TXCommand;
        PingRecvest (TXCommand &base) : _baseTX(base), kalman_t(1, 1, 0.1), kalman_aver(1, 1, 0.1)
        , kalman_aver2d(1, 1, 0.1), kalman_aver3d(1, 1, 0.1){
            _time_v.reserve(100);
            _3calibration.reserve(100);
            _aver_v.reserve(100);
        }

        // void TXCallBack();
        void PongCallBack(uint32_t pong);
    } pingrec;

    class TickRecvest {
        TXCommand& _baseTX;
        std::list<double> _time_v;
        double _aver;
        Kalman2<double> kalman_t;
        Kalman2<double> kalman_aver;
    public:
        friend TXCommand;
        TickRecvest (TXCommand &base) : _baseTX(base), kalman_t(5000, 0.99, 0.1), kalman_aver(1000, 0.99, 0.1)  {}

        void TickCallBack(uint32_t tick);
    } tickrec;
    

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

//    void PongCallBack(uint32_t pong);
    
    void serviceToSyncloop();
    void sendTrashPacket();
    void debugChannel();
    void sendToPingRecvest();
    void ICACHE_RAM_ATTR sendPing();
    void sendToTickRecvest();
    void toPingRecvest();
    void pingRecvest();
    void toTickRecvest();
    void wakeUp();
    void GPSRecvestloop();
    void sendGPSRecvest();
    void proccess();
    void TXPingDoneCallBack();
};