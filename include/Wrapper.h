#pragma once

#define API __attribute__((visibility("default")))
#define STD __attribute__((stdcall))

extern "C" {
  API void* STD initSLAM(char *strVocFile, char *strSettingsFile, int sensor, int saveMap);
  API void STD closeSLAM(void *System);
  API float * STD runSLAM(void *System, void *im, void *depth, int width, int height, double timestamp);
  API int STD statusSLAM(void *System);
  API int STD mapChangedSLAM(void *System);
  API void* STD initQRCodeTracker();
  API void STD loadQRCodes(void *QrCodeTracker);
  API void STD saveQRCodes(void *QrCodeTracker);
  API void STD track(void *QrCodeTracker, void *im, double x, double y);
}