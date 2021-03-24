#include <Arduino.h>
#include "TTGO_T-Beam_LoRa_APRS_config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>

#ifndef TASK_WEBSERVER
#define TASK_WEBSERVER


#define ENABLE_PREFERENCES

extern Preferences preferences;
#ifdef KISS_PROTOCOL
  extern WiFiServer tncServer;
#endif
// MAX 15 chars for preferenece key!!!
static const char *const PREF_WIFI_SSID = "wifi_ssid";
static const char *const PREF_WIFI_PASSWORD = "wifi_password";
static const char *const PREF_APRS_CALLSIGN = "aprs_callsign";
static const char *const PREF_APRS_RELAY_PATH = "aprs_relay_path";
static const char *const PREF_APRS_RELAY_PATH_INIT = "aprs_relay_init";
static const char *const PREF_APRS_SYMBOL_TABLE = "aprs_s_table";
static const char *const PREF_APRS_SYMBOL = "aprs_symbol";
static const char *const PREF_APRS_COMMENT = "aprs_comment";
static const char *const PREF_APRS_COMMENT_INIT = "aprs_comm_init";
static const char *const PREF_APRS_SHOW_ALTITUDE = "aprs_alt";
static const char *const PREF_APRS_SHOW_ALTITUDE_INIT = "aprs_alt_init";
static const char *const PREF_APRS_SHOW_BATTERY = "aprs_batt";
static const char *const PREF_APRS_SHOW_BATTERY_INIT = "aprs_batt_init";
static const char *const PREF_APRS_LATITUDE_PRESET = "aprs_lat_p";
static const char *const PREF_APRS_LATITUDE_PRESET_INIT = "aprs_lat_p_init";
static const char *const PREF_APRS_LONGITUDE_PRESET = "aprs_lon_p";
static const char *const PREF_APRS_LONGITUDE_PRESET_INIT = "aprs_lon_p_init";
static const char *const PREF_APRS_FIXED_BEACON_PRESET = "aprs_fixed_beac";
static const char *const PREF_APRS_FIXED_BEACON_PRESET_INIT = "aprs_fix_b_init";
static const char *const PREF_APRS_FIXED_BEACON_INTERVAL_PRESET = "aprs_fb_interv";
static const char *const PREF_APRS_FIXED_BEACON_INTERVAL_PRESET_INIT = "aprs_fb_in_init";
static const char *const PREF_APRS_GPS_EN = "gps_enabled";
static const char *const PREF_APRS_GPS_EN_INIT = "gps_state_init";
static const char *const PREF_APRS_SHOW_CMT = "show_cmt";
static const char *const PREF_APRS_SHOW_CMT_INIT = "show_cmt_init";
static const char *const PREF_DEV_BT_EN = "bt_enabled";
static const char *const PREF_DEV_BT_EN_INIT = "bt_enabled_init";
static const char *const PREF_DEV_OL_EN = "oled_enabled";
static const char *const PREF_DEV_OL_EN_INIT = "ol_enabled_init";
static const char *const PREF_DEV_SHOW_RX_TIME = "sh_rxtime";
static const char *const PREF_DEV_SHOW_RX_TIME_INIT = "sh_rxtime_init";
static const char *const PREF_DEV_AUTO_SHUT = "shutdown_act";
static const char *const PREF_DEV_AUTO_SHUT_INIT = "shutdown_actini";
static const char *const PREF_DEV_AUTO_SHUT_PRESET = "shutdown_dt";
static const char *const PREF_DEV_AUTO_SHUT_PRESET_INIT = "shutdown_dtini";

typedef struct {
  String callsign;
} tWebServerCfg;

[[noreturn]] void taskWebServer(void *parameter);
#endif