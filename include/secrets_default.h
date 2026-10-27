// secrets_default.h -- fallback if secrets.h is absent. Safe to commit.

#pragma once

#ifndef AP_SSID
  #define AP_SSID     "EVJ55-Shifter"
#endif
#ifndef AP_PASSWORD
  #define AP_PASSWORD "changeme"
#endif
#ifndef OTA_PASSWORD
  #define OTA_PASSWORD  "changeme"
#endif

// Home/shop WiFi -- M5Dial connects as client when available
#ifndef STA_SSID
#define STA_SSID        "your-home-ssid"
#endif

#ifndef STA_PASSWORD
#define STA_PASSWORD    "your-home-password"
#endif
