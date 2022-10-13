/**
 * TODO
 * - [x] Handle WiFi loss/reconnection
 * - [ ] Send logs via WS
 * - [x] Status leds
 * - [ ] AP physical ON/OFF switch?
 * - [ ] Display status in http?
 * - [ ] List AP clients in /config?
 * - [-] Enable TLS ~ requires valid certificate --> requires public hostname for acme validation
 * - [-] PWA ~ requires TLS
 */

/* Lib imports */
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFi.h>

#include "led-control-html.h"
#include "wifi-config-html.h"

/* Generic variables */
const char* deviceName = "<CHANGE-ME!>";                        // The AP SSID prefix/device hostname/mDNS host (.local)
uint8_t mac[6];                                                 // Will store the device MAC address

/* WiFi access point variables */
const char* apPassword = "<CHANGE-ME!>";                        // WiFI access point password
char apSsid[27];                                                // WiFI access point ssid
int apStatus = 0;                                               // The access point led status state (-1 blink, 0 low, 1 high)
const int apStatusGpio = 33;                                    // The GPIO for the WiFi access point status led
String apIPv6 = "0000:0000:0000:0000:0000:0000:0000:0000";      // Latest IPv6 assigned to WiFi the access point

/* WiFi station variables */
String stationSsid;                                             // WiFi Station SSID
String stationPassword;                                         // WiFi Station password
int reconnectDelayModifier = 0;                                 // Reconnect delay modifier
int stationStatus = 0;                                          // The station led status state(-1 blink, 0 low, 1 high)
const int stationStatusGpio = 27;                               // The GPIO for the WiFi status status led
String stationIPv6 = "0000:0000:0000:0000:0000:0000:0000:0000"; // Latest IPv6 assigned to WiFi the station

/* LED PWM variables */
/** static */
const int ledPwmChannel = 0;                                    // Ledstrip PWM channel
const int ledPwmFreq = 100;                                     // Ledstrip PWM frequency
const int ledPwmResolution = 12;                                // Ledstrip PWM Resolution
const int ledPwmGpio = 21;                                      // GPIO for PWM output
/** interactive */
bool areLedTurnedOn = 1;                                        // is the ledstrip ON
String ledPwmDutyCycle;                                         // PWM duty cycle

/* Serial variables */
const int serialBaudRate = 115200;                              // Baud rate for the serial connection

/* Web server variables */
const int webServerPort = 80;                                   // Web server port
AsyncWebServer webServer(webServerPort);                        // Web server instance
const char* controlIndex = CONTROL_page;                        // Ledstrip control UI code
const char* DUTY_CYCLE_PARAM = "dutycycle";                     // HTTP POST param key for the PWM duty cycle to adjust to
const char* configIndex = CONFIG_page;                          // WiFi config UI code
const char* SSID_PARAM = "ssid";                                // HTTP POST param key for the WiFI station ssid
const char* PASSWORD_PARAM = "password";                        // HTTP POST param key for the WiFi station password

// Preferences
Preferences preferences;                                        // Preferences instance
const char* DUTY_CYCLE_PREF = "duty-cycle";                     // PWM duty cycle preference key
const char* SSID_PREF = "ssid";                                 // WiFi station SSID preference key
const char* PASSWORD_PREF = "password";                         // WiFi station password preference key

/*
 * Replaces variables placeholders in web pages
 */
String processor(const String& var){
  if (var == "TURNEDON" && areLedTurnedOn == 1) {
    return "checked";
  }
  if (var == "DUTYCYCLE"){
    return ledPwmDutyCycle;
  }
  if (var == "SSID"){
    return stationSsid;
  }
  if (var == "PASSWORD"){
    return stationPassword;
  }
  return String();
}

/**
 * Computes the duty cycle based on the UI percentage and the PWM resolution
 */
int getDutyCycleValue(int resolution, int dutyPercentage) {
  double limited = (9 + ((double)dutyPercentage/100*(100-9)));  // minimum 6% duty cycle, let's say 10%
  double scaled = pow(2, (double)limited/100) - 1; // scale to correct non linear ramp up
  double inverted = 1 - scaled; // invert because of opto
  int maxDuty = pow(2, resolution);
  int dutyCycle = round((double)maxDuty * inverted);
  Serial.printf("[Compute DC] Computed duty cycle %d (received: %d%% - limited: %f - scaled: %f - corrected: %f)\n", dutyCycle, dutyPercentage, limited, scaled, inverted);
  return dutyCycle;
}

/**
 * Setups the GPIO for the status leds
 */
void setupStatusLeds() {
  pinMode(apStatusGpio, OUTPUT);
  pinMode(stationStatusGpio, OUTPUT);
}

/**
 * Loads previous saved state
 */
void loadPreferences() {
  preferences.begin("kitchen-leds", false);
  stationSsid = preferences.getString(SSID_PREF, ""); 
  stationPassword = preferences.getString(PASSWORD_PREF, "");
  ledPwmDutyCycle = preferences.getString(DUTY_CYCLE_PREF, "0");
}

/**
 * Saves the preferences state
 */
void savePrefrences() {
  preferences.clear();
  preferences.putString(SSID_PREF, stationSsid); 
  preferences.putString(PASSWORD_PREF, stationPassword);
  preferences.putString(DUTY_CYCLE_PREF, ledPwmDutyCycle);
}

/**
 * Configures leds PWM
 */
void setupPWM() {
  ledcAttachPin(ledPwmGpio, ledPwmChannel);
  ledcSetup(ledPwmChannel, ledPwmFreq, ledPwmResolution);
  if (areLedTurnedOn == 1) {
    ledcWrite(ledPwmChannel, getDutyCycleValue(ledPwmResolution, ledPwmDutyCycle.toInt()));
  }
  else {
    ledcWrite(ledPwmChannel, pow(2, ledPwmResolution));
  }
}

const char* system_event_names[] = { "WIFI_READY", "SCAN_DONE", "STA_START", "STA_STOP", "STA_CONNECTED", "STA_DISCONNECTED", "STA_AUTHMODE_CHANGE", "STA_GOT_IP", "STA_LOST_IP", "STA_WPS_ER_SUCCESS", "STA_WPS_ER_FAILED", "STA_WPS_ER_TIMEOUT", "STA_WPS_ER_PIN", "STA_WPS_ER_PBC_OVERLAP", "AP_START", "AP_STOP", "AP_STACONNECTED", "AP_STADISCONNECTED", "AP_STAIPASSIGNED", "AP_PROBEREQRECVED", "GOT_IP6", "ETH_START", "ETH_STOP", "ETH_CONNECTED", "ETH_DISCONNECTED", "ETH_GOT_IP", "MAX"};
const char* system_event_reasons[] = { "UNSPECIFIED", "AUTH_EXPIRE", "AUTH_LEAVE", "ASSOC_EXPIRE", "ASSOC_TOOMANY", "NOT_AUTHED", "NOT_ASSOCED", "ASSOC_LEAVE", "ASSOC_NOT_AUTHED", "DISASSOC_PWRCAP_BAD", "DISASSOC_SUPCHAN_BAD", "UNSPECIFIED", "IE_INVALID", "MIC_FAILURE", "4WAY_HANDSHAKE_TIMEOUT", "GROUP_KEY_UPDATE_TIMEOUT", "IE_IN_4WAY_DIFFERS", "GROUP_CIPHER_INVALID", "PAIRWISE_CIPHER_INVALID", "AKMP_INVALID", "UNSUPP_RSN_IE_VERSION", "INVALID_RSN_IE_CAP", "802_1X_AUTH_FAILED", "CIPHER_SUITE_REJECTED", "BEACON_TIMEOUT", "NO_AP_FOUND", "AUTH_FAIL", "ASSOC_FAIL", "HANDSHAKE_TIMEOUT", "CONNECTION_FAIL" };
#define reason2str(r) ((r>176)?system_event_reasons[r-176]:system_event_reasons[r-1])
/**
 * Handles WiFi events
 */
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.printf("[WiFi Event] %d - %s | ", event, system_event_names[event]);

  String newApIPv6;
  String newStaIPv6;
  switch (event) {
    case SYSTEM_EVENT_WIFI_READY: 
      Serial.println("WiFi interface ready");
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFi station started");
      postSTAStartConfig();
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.println("WiFi station stopped");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("WiFi station connected to access point");
      postSTAConnectedConfig();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.printf("WiFi station disconnected from WiFi access point: %d - %s\n", info.disconnected.reason, reason2str(info.disconnected.reason));
      stationStatus = 0;
      delay(reconnectDelayModifier * 10 * 1000);
      if(reconnectDelayModifier < 6) {
        reconnectDelayModifier++;
      }
      connectWifiStation();
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      Serial.println("WiFi station authentication mode of access point has changed");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("WiFi station obtained IP address: ");
      Serial.println(WiFi.localIP());
      stationStatus = 1;
      reconnectDelayModifier = 0;
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("WiFi station lost IP address and IP address is reset to 0");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WiFi station WiFi Protected Setup (WPS): succeeded in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WiFi station WiFi Protected Setup (WPS): failed in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WiFi station WiFi Protected Setup (WPS): timeout in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WiFi station WiFi Protected Setup (WPS): pin code in enrollee mode");
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.print("WiFi access point started with address: ");
      Serial.println(WiFi.softAPIP());
      postAPStartConfig();
      apStatus = 1;
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println("WiFi access point stopped");
      apStatus = 0;
      delay(1 * 60 * 1000);
      startWifiAccessPoint();
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.printf("Client connected to WiFi access point: %x:%x:%x:%x:%x:%x\n", info.sta_connected.mac[0], info.sta_connected.mac[1], info.sta_connected.mac[2], info.sta_connected.mac[3], info.sta_connected.mac[4], info.sta_connected.mac[5]);
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.printf("Client disconnected from WiFi access point: %x:%x:%x:%x:%x:%x\n", info.sta_disconnected.mac[0], info.sta_disconnected.mac[1], info.sta_disconnected.mac[2], info.sta_disconnected.mac[3], info.sta_disconnected.mac[4], info.sta_disconnected.mac[5]);
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      Serial.print("WiFi access point assigned IP address to client: ");
      Serial.println(IPAddress(info.ap_staipassigned.ip.addr).toString());
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      Serial.println("WiFi access point received probe request");
      break;
    case SYSTEM_EVENT_GOT_IP6:
      newApIPv6 = WiFi.softAPIPv6().toString();
      newStaIPv6 = WiFi.localIPv6().toString();
      if (apIPv6 != newApIPv6) {
        apStatus = 1;
        apIPv6 = newApIPv6;
        Serial.printf("WiFi access point IPv6: %s", apIPv6.c_str());
        if (stationIPv6 != newStaIPv6) {
          Serial.print(" - ");
        }
        /*else if (newStaIPv6 == "0000:0000:0000:0000:0000:0000:0000:0000" && !stationSsid.isEmpty()) {
          connectWifiStation();
        }*/
      }
      if (stationIPv6 != newStaIPv6) {
        stationStatus = 1;
        stationIPv6 = newStaIPv6;
        Serial.printf("WiFi station IPv6: %s", stationIPv6.c_str());
      }
      Serial.println("");
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("Ethernet started");
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("Ethernet stopped");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet connected");
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet disconnected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP address");
      break;
    default: 
      Serial.println("");
      break;
}}

const char* wifi_mode_names[] = { "NULL", "STATION", "ACCESS_POINT", "ACCESS_POINT_AND_STATION" };
/**
 * Setups WiFi config & event handlers
 */
void setupWiFi() {
  WiFi.onEvent(WiFiEvent);
  WiFi.macAddress(mac);
  WiFi.mode(WIFI_MODE_APSTA); // AP + Station   
  sprintf(apSsid, "%s-%x%x%x%x%x%x", deviceName, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  WiFi.setAutoReconnect(0);
  Serial.println("[WiFi Setup] WiFi setup:");
  Serial.printf("[WiFi Setup]   - mode: %d - %s\n", WiFi.getMode(), wifi_mode_names[WiFi.getMode()]);
  Serial.printf("[WiFi Setup]   - auto connect: %d\n", WiFi.getAutoConnect());
  Serial.printf("[WiFi Setup]   - auto reconnect: %d\n", WiFi.getAutoReconnect());
  Serial.printf("[WiFi Setup]   - device mac address: %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.printf("[WiFi Setup]   - access point name: %s\n", apSsid);
}

/**
 * Creates WiFi access point
 */
void startWifiAccessPoint() {
  apStatus = -1;
  Serial.printf("[WiFi AP] Starting Access Point '%s'\n", apSsid);
  WiFi.softAP(apSsid, apPassword);
}

/**
 * Connects the WiFi Station
 */
void connectWifiStation() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi STA] Disconnecting from current WiFi");
    WiFi.disconnect();
    return; // disconnect will trigger a new call to connectWifiStation
  }
  stationStatus = -1;
  Serial.printf("[WiFi STA] Connecting %x:%x:%x:%x:%x:%x to WiFi '%s'\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], stationSsid.c_str());
  WiFi.begin(stationSsid.c_str(), stationPassword.c_str());
}

/**
 * Configures WiFi access point after "AP_START" event
 */
void postAPStartConfig() {
  WiFi.softAPsetHostname(deviceName);
  WiFi.softAPenableIpV6();
  // with WIFI_MODE_APSTA mode, AP_START seems to be fired for STA_START as well...
  postSTAStartConfig();
}

/**
 * Configures WiFi station after "STA_START" event
 */
void postSTAStartConfig() {
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(deviceName);
}

/**
 * Configures WiFi station after "STA_CONNECTED" event
 */
void postSTAConnectedConfig() {
  WiFi.enableIpV6();
}

/**
 * Configures mDSN resolution
 */
void setupMDNS() {
  if(!MDNS.begin(deviceName)) {
     Serial.println("[mDNS] Error starting mDNS");
     return;
  }
  Serial.printf("[mDNS] Started mDNS as %s.local\n", deviceName);
}

/**
 * Starts the web server
 */
void setupWebServer() {
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("[Web Server] received GET '/'");
    request->send_P(200, "text/html", controlIndex, processor);
  });
  webServer.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("[Web Server] received GET '/config'");
    request->send_P(200, "text/html", configIndex, processor);
  });
  webServer.on("/save-credentials", HTTP_POST, [](AsyncWebServerRequest *request){
    Serial.println("[Web Server] received POST '/save-credentials'");
    if (!request->hasParam(SSID_PARAM, true) || !request->hasParam(PASSWORD_PARAM, true)) {
       Serial.println("[Web Server] '/save-credentials' error: no message sent");
       request->send(400, "text/plain", "Invalid parameters");
       return;
    }
    stationSsid = request->getParam(SSID_PARAM, true)->value();
    stationPassword = request->getParam(PASSWORD_PARAM, true)->value();
    Serial.printf("[Web Server] Received credentials for %s (%s)\n", stationSsid.c_str(), stationPassword.c_str());
    savePrefrences();
    connectWifiStation();
    request->redirect("/");
  });
  webServer.on("/ledpwm", HTTP_POST, [] (AsyncWebServerRequest *request) {
    Serial.println("[Web Server] received POST '/ledpwm'");
    if (!request->hasParam(DUTY_CYCLE_PARAM, true)) {
       Serial.println("[Web Server] '/ledpwm' error: no message sent");
       request->send(400, "text/plain", "Missing duty cycle");
       return;
    }
    ledPwmDutyCycle = request->getParam(DUTY_CYCLE_PARAM, true)->value();
    Serial.printf("[Web Server] Received duty cycle payload '%s'\n", ledPwmDutyCycle);
    ledcWrite(ledPwmChannel, getDutyCycleValue(ledPwmResolution, ledPwmDutyCycle.toInt()));
    savePrefrences();
    if (areLedTurnedOn == 0) {
      areLedTurnedOn = 1;
    }
    request->send(200, "text/plain", "OK");
  });
  webServer.on("/turnoff", HTTP_POST, [] (AsyncWebServerRequest *request) {
    Serial.println("[Web Server] received POST '/turnoff'");
    areLedTurnedOn = 0;
    ledcWrite(ledPwmChannel, pow(2, ledPwmResolution));
    request->send(200, "text/plain", "OK");
  }); 
  webServer.begin();
}

/**
 * Turns of/off or blink the status led based on the state
 */
void manageStatusLed(int gpio, int state) {
  if (state > -1) {
    digitalWrite(gpio, state);
    return;
  }
  digitalWrite(gpio, !digitalRead(gpio));
}

/**
 * Setup entry point
 */
void setup() {
  // Connect serial for debug
  Serial.begin(serialBaudRate);
  
  loadPreferences();
  setupStatusLeds();
  setupPWM();
  setupWiFi();
  startWifiAccessPoint();
  if (!stationSsid.isEmpty()) {
    connectWifiStation();
  }
  setupMDNS();
  setupWebServer();
}

/**
 * Main loop
 */
void loop() {
  manageStatusLed(apStatusGpio, apStatus);
  manageStatusLed(stationStatusGpio, stationStatus);
  delay(500);
}
