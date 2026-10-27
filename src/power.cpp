/*
 * power.cpp -- 12V loss detection, shutdown, WiFi AP, OTA
 *
 * WiFi runs in a separate FreeRTOS task so it cannot block or crash loop().
 * STA mode attempted first, falls back to AP if not connected in 10s.
 * OTA active when confirmed gear = Park.
 */

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include "globals.h"

#if __has_include("secrets.h")
  #include "secrets.h"
#endif
#include "secrets_default.h"

static volatile bool otaRunning = false;
static volatile bool staMode    = false;

bool otaActive() { return otaRunning; }
bool otaSTA()    { return staMode; }

// -- WiFi + OTA task -----------------------------------------------------------
static void wifiTask(void* p) {
    // Wait for system to stabilise before touching WiFi
    vTaskDelay(pdMS_TO_TICKS(3000));

    bool connected = false;

    // Try STA
    if (strlen(STA_SSID) > 0) {
        Serial.printf("WiFi: trying STA '%s'...\n", STA_SSID);
        WiFi.mode(WIFI_STA);
        WiFi.begin(STA_SSID, STA_PASSWORD);
        uint32_t start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
        if (WiFi.status() == WL_CONNECTED) {
            connected = true;
            staMode   = true;
            Serial.printf("WiFi STA IP: %s\n", WiFi.localIP().toString().c_str());
            if (MDNS.begin("evj55-shifter")) {
                Serial.println("mDNS: evj55-shifter.local");
            }
        } else {
            Serial.println("WiFi STA failed, trying AP");
            WiFi.disconnect(true);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // Fall back to AP
    if (!connected) {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        staMode = false;
        Serial.printf("WiFi AP '%s' IP: %s\n",
            AP_SSID, WiFi.softAPIP().toString().c_str());
    }

    // Setup OTA
    ArduinoOTA.setHostname("evj55-shifter");
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart([]()  { Serial.println("OTA start"); });
    ArduinoOTA.onEnd([]()    { Serial.println("OTA done"); });
    ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
        Serial.printf("OTA: %u%%\n", p * 100 / t);
    });
    ArduinoOTA.onError([](ota_error_t e) {
        Serial.printf("OTA error [%u]\n", e);
    });
    ArduinoOTA.begin();
    otaRunning = true;
    Serial.println("OTA ready");

    // Handle OTA forever
    for (;;) {
        if (otaRunning) {
            ArduinoOTA.handle();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static TaskHandle_t wifiTaskHandle = NULL;
static bool wifiTaskStarted = false;

// -- Called from loop() --------------------------------------------------------
void checkPowerState() {
    // Start WiFi task once when Park confirmed, never stop it
    // (stopping WiFi mid-session is risky; OTA handle runs regardless)
    bool inPark = (confirmed[0] == 0 && acked[0]);
    if (inPark && !wifiTaskStarted) {
        wifiTaskStarted = true;
        xTaskCreatePinnedToCore(wifiTask, "WIFI", 8192, NULL, 1, &wifiTaskHandle, 1);
    }

#if SHUTDOWN_TIMEOUT_S > 0
    static uint32_t lastCheck = 0;
    uint32_t now = millis();
    if (now - lastCheck < 2000) return;
    lastCheck = now;

    if (M5.Power.isCharging()) {
        if (powerLostMs != 0) {
            Serial.println("12V restored");
            powerLostMs = 0;
        }
    } else {
        if (powerLostMs == 0) {
            powerLostMs = now;
            Serial.println("12V lost - shutdown timer started");
        }
        uint32_t secsGone = (now - powerLostMs) / 1000;
        if (secsGone >= SHUTDOWN_TIMEOUT_S) {
            Serial.printf("Shutting down after %us on battery\n", secsGone);
            delay(100);
            M5.Power.powerOff();
        }
    }
#endif
}
