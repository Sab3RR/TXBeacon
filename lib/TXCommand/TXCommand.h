#include "common.h"
#include "OTA.h"
#include <string>
#include "FHSS.h"

namespace TXCommand {
    static bool ssResponce = false;
    static int syncResponceId = -1;

    static void setupFHSSChannel(const uint8_t channel)
    {
        Radio.SetFrequencyReg(FHSSgetCurrFreq(channel));
    }
    void serviceToSync();
    void loop();
    void encode (char c);
}