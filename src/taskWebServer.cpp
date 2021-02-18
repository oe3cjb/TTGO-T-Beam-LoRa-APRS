#include "taskWebServer.h"
/**
 * @see board_build.embed_txtfiles in platformio.ini
 */
extern const char web_index_html[] asm("_binary_data_embed_index_html_out_start");
extern const char web_index_html_end[] asm("_binary_data_embed_index_html_out_end");
extern const char web_style_css[] asm("_binary_data_embed_style_css_out_start");
extern const char web_style_css_end[] asm("_binary_data_embed_style_css_out_end");
extern const char web_js_js[] asm("_binary_data_embed_js_js_out_start");
extern const char web_js_js_end[] asm("_binary_data_embed_js_js_out_end");


String apSSID = "";
String apPassword = "xxxxxxxxxx";
WebServer server(80);
Preferences preferences;
#ifdef KISS_PROTOCOL
  WiFiServer tncServer(NETWORK_TNC_PORT);
#endif

void sendCacheHeader() { server.sendHeader("Cache-Control", "max-age=3600"); }
void sendGzipHeader() { server.sendHeader("Content-Encoding", "gzip"); }

String jsonEscape(String s){
  s.replace("\"", "\\\"");
  s.replace("\\", "\\\\");
  return s;
}

String jsonLineFromPreferenceString(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":\"" + jsonEscape(preferences.getString(preferenceName)) + (last ?  + R"(")" :  + R"(",)");
}
String jsonLineFromPreferenceBool(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":" + (preferences.getBool(preferenceName) ? "true" : "false") + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromPreferenceInt(const char *preferenceName, bool last=false){
  return String("\"") + preferenceName + "\":" + (preferences.getInt(preferenceName)) + (last ?  + R"()" :  + R"(,)");
}
String jsonLineFromString(const char *name, const char *value, bool last=false){
  return String("\"") + name + "\":" + jsonEscape(value) + (last ?  + R"()" :  + R"(,)");
}

void handle_NotFound(){
  sendCacheHeader();
  server.send(404, "text/plain", "Not found");
}

void handle_Index() {
  sendGzipHeader();
  server.send_P(200, "text/html", web_index_html, web_index_html_end - web_index_html);
}

void handle_Style() {
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/css", web_style_css, web_style_css_end - web_style_css);
}

void handle_Js() {
  sendCacheHeader();
  sendGzipHeader();
  server.send_P(200, "text/javascript", web_js_js, web_js_js_end-web_js_js);
}

void handle_ScanWifi() {
  String listResponse = R"(<label for="networks_found_list">Networks found:</label><select class="u-full-width" id="networks_found_list">)";
  int n = WiFi.scanNetworks();
  listResponse += "<option value=\"\">Select Network</option>";

  for (int i = 0; i < n; ++i) {
    listResponse += "<option value=\""+WiFi.SSID(i)+"\">" + WiFi.SSID(i) + "</option>";
  }
  listResponse += "</select>";
  server.send(200,"text/html", listResponse);
}

void handle_SaveWifiCfg() {
  if (!server.hasArg("wifi_ssid") || !server.hasArg("wifi_password")){
    server.send(500, "text/plain", "Invalid request");
  }
  if (!server.arg("wifi_ssid").length() || !server.arg("wifi_password").length()){
    server.send(403, "text/plain", "Empty SSID or Password");
  }

  preferences.putString("wifi_ssid", server.arg("wifi_ssid"));
  preferences.putString("wifi_password", server.arg("wifi_password"));

  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
}

void handle_Reboot() {
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  ESP.restart();
}

void handle_Restore() {
  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");
  preferences.clear();
  preferences.end();
  ESP.restart();
}

void handle_Cfg() {
  String jsonData = "{";
  jsonData += R"("wifi_ssid":")" + jsonEscape(preferences.getString("wifi_ssid")) + R"(",)";
  jsonData += R"("wifi_password":")" + jsonEscape((preferences.getString("wifi_password").isEmpty() ? String("") : "*")) + R"(",)";
  jsonData += jsonLineFromPreferenceString(PREF_APRS_CALLSIGN);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_RELAY_PATH);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_SYMBOL_TABLE);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_SYMBOL);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_COMMENT);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_LATITUDE_PRESET);
  jsonData += jsonLineFromPreferenceString(PREF_APRS_LONGITUDE_PRESET);
  jsonData += jsonLineFromPreferenceInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_BATTERY);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_FIXED_BEACON_PRESET);
  jsonData += jsonLineFromPreferenceBool(PREF_APRS_SHOW_ALTITUDE);
  jsonData += jsonLineFromString("FreeHeap", String(ESP.getFreeHeap()).c_str());
  jsonData += jsonLineFromString("HeapSize", String(ESP.getHeapSize()).c_str());
  jsonData += jsonLineFromString("FreeSketchSpace", String(ESP.getFreeSketchSpace()).c_str(), true);

  jsonData += "}";
  server.send(200,"application/json", jsonData);
}

void handle_SaveAPRSCfg() {
  if (server.hasArg(PREF_APRS_CALLSIGN) && !server.arg(PREF_APRS_CALLSIGN).isEmpty()){
    preferences.putString(PREF_APRS_CALLSIGN, server.arg(PREF_APRS_CALLSIGN));
  }
  if (server.hasArg(PREF_APRS_SYMBOL_TABLE) && !server.arg(PREF_APRS_SYMBOL_TABLE).isEmpty()){
    preferences.putString(PREF_APRS_SYMBOL_TABLE, server.arg(PREF_APRS_SYMBOL_TABLE));
  }
  if (server.hasArg(PREF_APRS_SYMBOL) && !server.arg(PREF_APRS_SYMBOL).isEmpty()){
    preferences.putString(PREF_APRS_SYMBOL, server.arg(PREF_APRS_SYMBOL));
  }
  if (server.hasArg(PREF_APRS_RELAY_PATH)){
    preferences.putString(PREF_APRS_RELAY_PATH, server.arg(PREF_APRS_RELAY_PATH));
  }
  if (server.hasArg(PREF_APRS_COMMENT)){
    preferences.putString(PREF_APRS_COMMENT, server.arg(PREF_APRS_COMMENT));
  }
  if (server.hasArg(PREF_APRS_LATITUDE_PRESET)){
    preferences.putString(PREF_APRS_LATITUDE_PRESET, server.arg(PREF_APRS_LATITUDE_PRESET));
  }
  if (server.hasArg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET)){
    preferences.putInt(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET, server.arg(PREF_APRS_FIXED_BEACON_INTERVAL_PRESET).toInt());
  }
  if (server.hasArg(PREF_APRS_LONGITUDE_PRESET)){
    preferences.putString(PREF_APRS_LONGITUDE_PRESET, server.arg(PREF_APRS_LONGITUDE_PRESET));
  }
  preferences.putBool(PREF_APRS_SHOW_BATTERY, server.hasArg(PREF_APRS_SHOW_BATTERY));
  preferences.putBool(PREF_APRS_SHOW_ALTITUDE, server.hasArg(PREF_APRS_SHOW_ALTITUDE));
  preferences.putBool(PREF_APRS_FIXED_BEACON_PRESET, server.hasArg(PREF_APRS_FIXED_BEACON_PRESET));


  server.sendHeader("Location", "/");
  server.send(302,"text/html", "");

}

[[noreturn]] void taskWebServer(void *parameter) {
  auto *webServerCfg = (tWebServerCfg*)parameter;
  apSSID = webServerCfg->callsign + " AP";

  server.on("/", handle_Index);
  server.on("/favicon.ico", handle_NotFound);
  server.on("/style.css", handle_Style);
  server.on("/js.js", handle_Js);
  server.on("/scan_wifi", handle_ScanWifi);
  server.on("/save_wifi_cfg", handle_SaveWifiCfg);
  server.on("/reboot", handle_Reboot);
  server.on("/cfg", handle_Cfg);
  server.on("/save_aprs_cfg", handle_SaveAPRSCfg);
  server.on("/restore", handle_Restore);
  server.onNotFound(handle_NotFound);

  String wifi_password = preferences.getString("wifi_password");
  String wifi_ssid = preferences.getString("wifi_ssid");
  if (!wifi_password.length() || !wifi_ssid.length()){
    WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  } else {
    WiFi.begin(wifi_ssid.c_str(), wifi_password.c_str());
    Serial.println("Connecting to " + wifi_ssid);
    while (WiFi.status() != WL_CONNECTED) {
      Serial.println("Not connected");
      vTaskDelay(500/portTICK_PERIOD_MS);
    }
  }

  server.begin();
  #ifdef KISS_PROTOCOL
    tncServer.begin();
  #endif
  if (MDNS.begin(webServerCfg->callsign.c_str())) {
    MDNS.setInstanceName(webServerCfg->callsign + " TTGO LoRa APRS TNC " + TXFREQ + "MHz");
    MDNS.addService("http", "tcp", 80);
    #ifdef KISS_PROTOCOL
      MDNS.addService("kiss-tnc", "tcp", NETWORK_TNC_PORT);
    #endif
  }

  while (true){
    server.handleClient();
    vTaskDelay(5/portTICK_PERIOD_MS);
  }
}