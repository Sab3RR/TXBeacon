#include "common.h"
#include "OTA.h"
#include <string>
#include "FHSS.h"

class TXCommand {
    static bool get_ssResponce();
    static void set_ssResponce(bool s);
    static int get_syncRespId();
    static void set_syncRespId(int s);
    static bool get_grResp();
    static void set_grResp(bool s);
    static void inc_gpsIter();
    static double get_lat();
    static double get_lng();
    static double get_alt();
    static void set_1lat(uint32_t s);
    static void set_2lat(uint32_t s);
    static void set_1lng(uint32_t s);
    static void set_2lng(uint32_t s);
    static void set_1alt(uint32_t s);
    static void set_2alt(uint32_t s);

    static void setupFHSSChannel(const uint8_t channel)
    {
        Radio.SetFrequencyReg(FHSSgetCurrFreq(channel));
    }
    void serviceToSync();
    void loop();
    void encode (char c);
};