#pragma once

#define API __attribute__((visibility("default")))
#define STD __attribute__((stdcall))

extern "C" {
  API void* STD initSLAM(char *strVocFile, char *strSettingsFile, int sensor, int saveMap);
  API void STD closeSLAM(void *System);
  API double * STD runSLAM(void *System, void *im, void *depth, int width, int height, double timestamp);
}