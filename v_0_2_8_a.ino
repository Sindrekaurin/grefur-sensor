#include <EEPROM.h>
#include <PubSubClient.h>
#include <Base64.h>
#include <math.h>


#define CONFIG_START 0
#define CONFIG_SIZE sizeof(Config)
#define SENSOR_CONFIG_START (CONFIG_START + CONFIG_SIZE)

#define SENSOR_CONFIG_SIZE (EEPROM_SIZE - SENSOR_CONFIG_START) / MAX_SENSORS
#define SENSOR_NAME_MAX_LENGTH 16

#define INPUT_MODULE true
#define OUTPUT_MODULE false


#define MAX_SENSORS_ALLOWED 4
#define MAX_ACTUATORS_ALLOWED 4
#define MAX_CALCULATIONS_ALLOWED 10


#define DEBUG false
#define DEBUG_SYSTEM false
#define DEBUG_ERROR_VISUALIZATION false
#define DEBUG_ACTUATOR_MANAGER true
#define DEBUG_MUX_FUNCTIONALITY false
#define DEBUG_MQTT_MANAGER false
#define DEBUG_NETWORK_CONNECTION false
#define DEBUG_EXPRESSIONS false
#define DEBUG_BINDING_ENGINE false

#define USE_HTTPS false
#define BASETOPIC_DEFAULT "values"

// Make networksettings interface and connection method change
#define WIRELESS_NETWORK true


#define MUX_A 2
#define MUX_B 14

unsigned long lastPublishTime = 0;
bool winkActive = false;
bool activeError = true;
unsigned long lastMQTTReconnectAttempt = 0;

constexpr char FIRMWARE_VERSION[] = "0.3.8a";
const char* GREFUR_BACKEND_URL = "http://192.168.10.108:5000";




#if defined(ESP8266)
#include <ArduinoJson.h>
#include <FS.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h> // Used for http
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServerSecure.h> // Used for https
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
//ESP8266WebServer server(443);
ESP8266WebServer server(80);

#define ONBOARD_LED LED_BUILTIN
#define ONBOARD_LED_ON  LOW   // ACTIVE on LOW
#define ONBOARD_LED_OFF HIGH  // LED OFF


#elif defined(ESP32)

#if defined(ARDUINO_XIAO_ESP32C3)
#define ONBOARD_LED 8
#elif defined(ARDUINO_XIAO_ESP32S3) || defined(ARDUINO_ESP32S3_DEV)
#define ONBOARD_LED 7
#else
#define ONBOARD_LED 2 // Standard fallback for de fleste andre ESP32
#endif
#define ONBOARD_LED_ON  HIGH //
#define ONBOARD_LED_OFF LOW // 



#include <Update.h>
#include <WebServer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <HTTPUpdateServer.h>
#include <ArduinoOTA.h>


HTTPUpdateServer httpUpdater;

#if USE_HTTPS
#include <BearSSLHelpers.h>
WebServer server(443);
#else
WebServer server(80);
#endif



const char* serverKey = R"KEY(-----BEGIN PRIVATE KEY-----

-----END PRIVATE KEY-----
)KEY";

const char* serverCert = R"CERT(
-----BEGIN CERTIFICATE-----

-----END CERTIFICATE-----
)CERT";


/*void setupOTA() {
    ArduinoOTA.setHostname("xiao-sensor");  // valgfritt
    ArduinoOTA.setPassword("hemmelig123");  // anbefalt

    ArduinoOTA.onStart([]() {
        log_d("OTA start");
    });
    ArduinoOTA.onEnd([]() {
        log_d("OTA ferdig");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        log_e("OTA feil: %u", error);
    });

    ArduinoOTA.begin();
}*/



#elif defined(STM32F7) || defined(STM32H7)
EthernetServer server(80);    // Use EthernetServer for STM32
#else
#error "Unsupported board!"
#endif

#if WIRELESS_NETWORK
WiFiClient netClient; // WiFi TCP client
#else
#include <Ethernet.h>
EthernetClient netClient;           // Ethernet TCP client
#endif

PubSubClient mqttClient(netClient);



// x.y.z.o
// x -> New hardware
// y -> New software
// z -> Changes
// o -> Operation state Ex alpha, beta, prod




/**
   Default admin:
    Username: admin
    Password: admin

   Running:
    While available wifi:
      Connecting to access point
      Load settings
      Starts broadcasting to broker
      Settings can be canged at given IP adress
    While not available wifi:
      Creates its own access point
      Load settings if existing
      Access it with connecting to its wifi endpoint
      Change settings for wifi and mqtt
      Save and restart module

     Version log:
      0_0_2:
        Firmware over internet
        Introduction of version control

*/

String generateAPName() {
  #if defined(ESP8266)
  return "Grefur_" + String(ESP.getChipId(), HEX);
  #elif defined(ESP32)
  uint64_t chipId = ESP.getEfuseMac();
  uint32_t shortId = (uint32_t)(chipId >> 32);
  return "Grefur_" + String(shortId, HEX);
  #endif

}

#pragma pack(push, 1) // Exact byte alignment

/******************************
          HEADER SECTION
 ******************************/
struct EEPROMHeader {
  uint32_t magic = 0xAA55AA55;
  char version[16] = {};
  uint8_t checksum;
  uint8_t _reserved[10];
};

/******************************
       DEVICE METADATA
 ******************************/
struct DeviceMetadata {
  char apName[32];              // Generate this during initialization
  char deviceType[16] = "Backplate";
  char serialNum[32];           // Populated during init
  char productionToken[32];
  char deviceName[32] = "Enhetsnavn";
  char location[32] = "Lokasjon";
  char adminUser[16] = "admin";
  char adminPass[16] = "admin";
};

/******************************
      NETWORK SETTINGS
******************************/
struct NetworkSettings {
  // WiFi settings
  char wifiSSID[32];
  char wifiPassword[32];

  // Ethernet settings
  bool useDHCP;        // 1 byte: true = use DHCP, false = static IP
  // Creates 3 unused bytes for padding

  IPAddress staticIP;  // 4 bytes: static IP if useDHCP is false
  IPAddress gateway;   // 4 bytes: gateway for Ethernet
  IPAddress subnet;    // 4 bytes: subnet mask
  //uint8_t _padding[3]; // 3 bytes padding to keep struct size same as original 16-byte reserved
};


/******************************
         MQTT CONFIG
 ******************************/
struct MQTTConfig {
  char brokerURL[64];
  uint16_t port = 1883;
  char clientID[32];
  char username[32];
  char password[32];
  char baseTopic[64];
  uint8_t qos = 1;
  bool retain = true;
  bool useNestedJson = true;
  bool useGrefurBackend = true;
  uint8_t _reserved[2];         // Padding
};

/******************************
      OPERATION SETTINGS
 ******************************/
struct OperationSettings {
  uint32_t publishInterval = 60000;
  float minValidTemp = -50.0;
  float maxValidTemp = 120.0;
  bool publishMeasuredValues = true;
  bool publishIP = true;
  bool publishStatus = true;
  bool publishDeviceInfo = true;
  bool publishRSSI = false;
  bool publishWatchdog = true;
  uint8_t _reserved[1];         // Padding
};


/******************************
      PUBLISH TRIGGER CONFIG
 ******************************/

enum PublishTrigger : uint8_t {
  PUBLISH_ON_THRESHOLD_OR_INTERVAL = 0,  // Default: publish if change >= threshold OR interval elapsed
  PUBLISH_ON_CHANGE_OF_THRESHOLD   = 1,  // Only publish if change >= threshold (ignore interval)
  PUBLISH_ON_INTERVAL              = 2,  // Only publish on fixed interval (ignore threshold)
  PUBLISH_ON_THRESHOLD_OR_INTERVAL_= 0,  // Alias, same as default
};

struct PublishPolicy {
  PublishTrigger trigger = PUBLISH_ON_THRESHOLD_OR_INTERVAL;
  float          changeThreshold  = 0.5f;   // Min delta to count as "changed" (units match sensor)
  uint32_t       intervalMs       = 60000;  // Publish interval in ms (used when trigger includes interval)
  uint8_t        _reserved[3];              // Padding to 12 bytes total
};

/******************************
      SENSOR CONFIGURATION
 ******************************/
struct SensorConfig {
  int8_t type = -1;                   // Sensor type identifier
  char name[16];                 // User-assigned name
  union {
    struct {
      float seriesResistor;      // Ω (Ohms)
      float nominalResistance;   // Ω (Ohms)
      float nominalTemp;         // °C (Celsius)
      float bCoefficient;        // K (Kelvin)
      int analogPin;
      char unit[8];              // Temperature unit (default Celsius)
    } ntc;
    struct {
      int analogPin;
      float inputMin;            // V (Volts)
      float inputMax;            // V (Volts)
      float scaleMin;            // Minimum scaled value
      float scaleMax;            // Maximum scaled value
      char scaleUnit[8];         // Unit for scaled values (e.g., "bar", "kPa")
    } analog;
    struct {
      int digitalPin;
      bool inverted;
    } digital;
  };
  float lastMeasurement;         // runtime-only value
  uint32_t lastUpdateTs;      // runtime-only — timestamp of last publish
  PublishPolicy publishPolicy;
  uint8_t _reserved[14];
};

enum SensorType : int8_t {
  SENSOR_UNCONFIGURED = -1,
  SENSOR_NTC = 0,               // Temperature sensor
  SENSOR_ANALOG,                // Analog voltage/current sensor
  SENSOR_DIGITAL,                // Digital on/off sensor
};

/* Configuration of caluclations */

/* Summary of function: Configuration for data processing slots, supporting local expressions or external bindings */
struct CalculationConfig {
  int8_t type;              // Uses CalculationType enum
  char name[16];            // Friendly name for the UI/Broker
  char unit[8];              // e.g., "°C", "%", "lux"
  bool publishValue;        // Whether to push results back to the broker
  PublishPolicy publishPolicy;

  union {
    /* Logical triggers or boolean results based on math */
    struct {
      char expression[64];
      uint8_t evalMode;
      float triggerThreshold;
      uint32_t evalIntervalMs;
    } expression;

    /* Continuous mathematical transformations */
    struct {
      char expression[64];
      uint32_t evalIntervalMs;
      float minValue;
      float maxValue;
    } continuous;

    /* Input Binding: Flexible reference to internal or external data */
    struct {
      float lastValue;
      char tag[48];             // The ID or Full Topic string
      float fallbackValue;

      // Subscription logic
      bool isNested; // Flag: If true, parse as JSON; if false, parse as flat value
      char nestedValueKey[32];    // The specific key to extract if isNested is true

      // Dynamic Variable Logic
      uint32_t resetIntervalMs; // Literal ms OR ID for reference lookup
      bool isResetIntervalRef;  // Flag: if true, look up resetIntervalMs elsewhere

      // Freshness & Performance tracking
      uint32_t lastUpdateTs;    // Timestamp of last MQTT/internal arrival
      uint32_t lastScanTimeMs;  // Microcontroller execution time for this slot
      float minValObserved;     // Historical low since last reset
      float maxValObserved;     // Historical high since last reset
    } binding;
  };
};

enum CalculationType : int8_t {
  CALC_EXPRESSION = 1,
  CALC_CONTINUOUS = 2,
  CALC_EXTERNAL   = 3  // Represents the 'binding' struct
};

enum ExpressionEvalMode : uint8_t {
  EVAL_ON_CHANGE   = 0,
  EVAL_ON_INTERVAL = 1
};

struct ActuatorConfig {
  int8_t type = -1;               // Actuator type identifier
  char name[16];                  // User-assigned name

  union {
    struct {
      int digitalPin;         // GPIO pin for digital output
      bool inverted;          // Invert logic
    } digital;

    struct {
      int analogPin;          // PWM/DAC pin
      float minValue;         // Minimum value (0%)
      float maxValue;         // Maximum value (100%)
      char unit[8];           // Unit (%, V, etc.)
      float voltageMin;    // Phsycial voltage
      float voltageMax;    // Physical voltage
    } analog;
  };

  PublishPolicy publishPolicy;     // 12 bytes

  // Binding properties
  char sourceName[32];        // Name of the Expression

  // Runtime data (RAM only, not for EEPROM/Flash storage)
  float runtimeValue;         // Last applied value to hardware
  uint32_t lastUpdateTs;      // Timestamp of last hardware write
  float lastPublishedValue;   // Last value sent to MQTT
  uint32_t lastPublishTs;     // Timestamp of last MQTT publish
  uint8_t _reserved[4];
};

enum ActuatorType : int8_t {
  ACTUATOR_UNCONFIGURED = -1,
  ACTUATOR_DIGITAL = 0,           // Digital on/off actuator
  ACTUATOR_ANALOG = 1             // Analog actuator (PWM/DAC)
};


/*
  struct CertificateStorage {
  char cert[2048];       // Device certificate (PEM text)
  char key[2048];        // Private key (PEM text)
  char rootCA[2048];     // Root CA certificate
  uint32_t issued;       // UNIX timestamp
  uint32_t expires;      // UNIX timestamp
  };

*/


/******************************
    COMPLETE EEPROM LAYOUT
 ******************************/
struct EEPROMLayout {
  EEPROMHeader header;
  DeviceMetadata device;
  NetworkSettings network;
  MQTTConfig mqtt;
  OperationSettings operation;
  SensorConfig sensors[MAX_SENSORS_ALLOWED];
  ActuatorConfig actuators[MAX_ACTUATORS_ALLOWED];
  CalculationConfig calculations[MAX_CALCULATIONS_ALLOWED];
  //CertificateStorage certificates;
};

#pragma pack(pop) // Restore default packing


float lastValidTemp = 0;
unsigned long lastPublish = 0;
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000;


EEPROMLayout config;  // Global configuration object

#define EEPROM_SIZE sizeof(EEPROMLayout)  // Pre-defined eeprom size

static_assert(sizeof(EEPROMLayout) <= EEPROM_SIZE, "EEPROMLayout er større enn EEPROM_SIZE!");

void testConfigIntegrity() {
  Serial.println("Testing config integrity...");

  // Check if config is loaded
  if (config.header.magic != 0xAA55AA55) {
    Serial.println("Config not loaded or invalid!");
    return;
  }

  Serial.printf("Firmware version: '%s'\n", config.header.version);
  Serial.printf("Device name: '%s'\n", config.device.deviceName);
  Serial.printf("WiFi SSID: '%s'\n", config.network.wifiSSID);
  // Add more fields as needed
}

uint8_t calculateChecksum(const uint8_t* data, size_t size) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < size; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Helper functions for EEPROM storage
bool saveConfig() {
  // Use the global config object instead of creating a new one

  // Update checksum (temporarily clear it for calculation)
  config.header.checksum = 0;
  config.header.checksum = calculateChecksum((uint8_t*)&config, sizeof(config));

  EEPROM.begin(sizeof(EEPROMLayout));
  EEPROM.put(0, config);  // Save the GLOBAL config
  if (!EEPROM.commit()) {
    Serial.println("EEPROM commit failed!");
    return false;
  } else {
    EEPROM.end();
    Serial.println("Config saved successfully");
    return true;
  }
}

bool loadConfig() {
  EEPROM.begin(sizeof(EEPROMLayout));
  EEPROM.get(0, config);  // Load into GLOBAL config
  EEPROM.end();

  // Validate
  if (config.header.magic != 0xAA55AA55) {
    Serial.println("Invalid magic number - using default config");
    return false;
  }

  uint8_t saved_checksum = config.header.checksum;
  config.header.checksum = 0; // Reset before calculating

  if (calculateChecksum((uint8_t*)&config, sizeof(config)) != saved_checksum) {
    Serial.println("Checksum mismatch - config may be corrupted");
    return false;
  }

  // Restore the checksum
  config.header.checksum = saved_checksum;

  Serial.println("Config loaded successfully");
  return true;
}

/* Middelware / Logic between the server and the client,
 *  */

/**
  BASIC HTTP AUTH
  Verify correct username and password
*/
bool isAuthenticated() {
  if (strlen(config.device.adminPass) == 0) return true;
  return server.authenticate("admin", config.device.adminPass);
}


String getFirmwareVersion() {
  return String(FIRMWARE_VERSION);
}



// Middleware to check authentication
bool authenticateRequest() {
  if (!isAuthenticated()) {
    server.send(401, "text/plain", "Unauthorized");
    return false;
  }
  return true;
}

/*
   @brief check if new pin in use, then send warning
*/

bool isPinUsed(uint8_t pin) {
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    if (config.sensors[i].type == SENSOR_NTC && config.sensors[i].ntc.analogPin == pin) return true;
    if (config.sensors[i].type == SENSOR_ANALOG && config.sensors[i].analog.analogPin == pin) return true;
    if (config.sensors[i].type == SENSOR_DIGITAL && config.sensors[i].digital.digitalPin == pin) return true;
  }
  return false;
}

const uint8_t* generateMacFromAP(const char* apName) {
  static uint8_t mac[6]; // static so it persists after function returns

  mac[0] = 0x02; // Locally administered, unicast

  // Simple hash for uniqueness
  uint32_t hash = 0;
  for (int i = 0; apName[i] != 0; i++) {
    hash = hash * 31 + apName[i];
  }

  // Fill remaining 5 bytes from hash
  mac[1] = (hash >> 24) & 0xFF;
  mac[2] = (hash >> 16) & 0xFF;
  mac[3] = (hash >> 8) & 0xFF;
  mac[4] = hash & 0xFF;

  // Mix in AP characters for last byte
  uint8_t last = 0;
  for (int i = 0; apName[i] != 0; i++) {
    last ^= apName[i];
  }
  mac[5] = last;

  return mac;
}






/*
   @ brief: Handler for gateway parametersavings. All parameters for connections
*/

class SaveConfigManager {
  public:
    SaveConfigManager(EEPROMLayout& config) : _config(config), _changed(false) {}

    // Middleware to check authentication
    bool authenticateRequest() {
      if (!isAuthenticated()) {
        server.send(401, "text/plain", "Unauthorized");
        return false;
      }
      return true;
    }

    // Update methods
    bool updateStringField(char* dest, size_t destSize, const String& newValue) {
      if (newValue.length() > 0 && strcmp(dest, newValue.c_str()) != 0) {
        strncpy(dest, newValue.c_str(), destSize);
        dest[destSize - 1] = '\0';
        return (_changed = true);
      }
      return false;
    }

    template <typename T>
    bool updateNumericField(T& dest, const String& newValue) {
      if (newValue.length() > 0) {
        T newVal;
        if constexpr (std::is_same_v<T, int>) newVal = newValue.toInt();
        else if constexpr (std::is_same_v<T, long>) newVal = newValue.toInt();
        else if constexpr (std::is_same_v<T, int8_t>) newVal = static_cast<int8_t>(newValue.toInt());
        else if constexpr (std::is_same_v<T, float>) newVal = newValue.toFloat();
        else if constexpr (std::is_same_v<T, uint32_t>) newVal = static_cast<uint32_t>(newValue.toInt());
        else if constexpr (std::is_same_v<T, uint16_t>) newVal = static_cast<uint16_t>(newValue.toInt());
        else if constexpr (std::is_same_v<T, uint8_t>) newVal = static_cast<uint8_t>(newValue.toInt());
        else {
          if (DEBUG) {
            Serial.println("Given type is not supported");
          }
          return false;
        }

        if (dest != newVal) {
          dest = newVal;
          if (DEBUG) {
            Serial.print("Value updated to: ");
            Serial.println(newVal);
          }
          return (_changed = true);
        } else {
          if (DEBUG) {
            Serial.println("Value is the same, no update needed");
          }
        }
      } else {
        if (DEBUG) {
          Serial.println("Given value has zero length");
        }
      }
      return false;
    }




    bool updateUint16Field(uint16_t& dest, const String& newValue) {
      if (newValue.length() > 0) {
        uint16_t newVal = static_cast<uint16_t>(newValue.toInt());
        if (dest != newVal) {
          dest = newVal;
          return (_changed = true);
        }
      }
      return false;
    }

    bool stringToBoolValue(const String& str) {
      String s = str;
      s.toLowerCase();
      return (s == "1" || s == "true" || s == "on" || s == "yes");
    }



    bool updateBoolField(bool& dest, bool newValue) {
      if (dest != newValue) {
        dest = newValue;
        return (_changed = true);
      }
      return false;
    }

    bool updateIPAddressField(IPAddress& dest, const String& newValue) {
      if (newValue.length() == 0) return false;

      IPAddress newIP;
      int parts[4] = {0};

      // Parse string like "192.168.1.100"
      if (sscanf(newValue.c_str(), "%d.%d.%d.%d", &parts[0], &parts[1], &parts[2], &parts[3]) != 4) {
        if (DEBUG) Serial.println("Invalid IP format");
        return false;
      }

      for (int i = 0; i < 4; i++) {
        if (parts[i] < 0 || parts[i] > 255) {
          if (DEBUG) Serial.println("IP part out of range");
          return false;
        }
        newIP[i] = parts[i];
      }

      // Compare with current value
      if (dest != newIP) {
        dest = newIP;
        if (DEBUG) Serial.print("IP updated to: "); Serial.println(dest);
        return (_changed = true);
      }

      return false;
    }

    // This triggers restart if called before finalize
    void scheduleRestart() {
      _shouldRestart = true;
    }

    void performRestart(){
      if (server.client()) { // Ensure client is existing
        server.client().stop(); // Ensure client connection is closed
      }
      delay(500);              // Short delay to ensure response is sent
      ESP.restart();           // Restart the ESP
      delay(500);
    }

    void setSendHtml(bool flag) {
      _sendHtml = flag;
    }

    /* Summary of function: Finalizes configuration changes, optionally sends web response, and handles scheduled restarts */
    void finalize(const String& successMessage = "Settings updated") {
      if (_changed) {
        saveConfig(); // Persist to EEPROM
        if (DEBUG) Serial.println("[Config] Changes saved to EEPROM.");
      }

      // Only attempt to send web response if we are in a web context
      // and a client is actually connected to the web server
      if (_sendHtml && server.client()) {
        String response = _changed ? successMessage : "No changes detected";
        if (_shouldRestart) response += ". Device restarting...";

        String html = "<p>" + response + "</p>";
        server.send(200, "text/html", html);
        delay(100);
      }

      if (_shouldRestart) {
        if (DEBUG) Serial.println("[Config] Restart scheduled. Rebooting now...");
        performRestart();
      }

      _changed = false;
      _shouldRestart = false;
    }


    bool hasChanges() const {
      return _changed;
    }

  private:
    EEPROMLayout& _config;
    bool _changed;
    bool _shouldRestart;  // New flag for restart scheduling
    bool _sendHtml = false;
};



/*
 * HELPERFUNCTIONS
 *
 *
 */

void printWiFiStatus() {
  switch (WiFi.status()) {
    case WL_IDLE_STATUS:    Serial.println("Idle"); break;
    case WL_NO_SSID_AVAIL:  Serial.println("SSID not found"); break;
    case WL_CONNECT_FAILED: Serial.println("Wrong password"); break;
    case WL_CONNECTION_LOST: Serial.println("Connection lost"); break;
    case WL_DISCONNECTED:   Serial.println("Disconnected"); break;
    default:                Serial.printf("Error %d\n", WiFi.status());
  }
}

void printEEPROMStatus() {
  Serial.println("\nEEPROM Status:");
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    Serial.printf("Sensor %d: type=%d, name='%s'\n",
                  i, config.sensors[i].type, config.sensors[i].name);
  }
}


void asyncResponse(const std::vector<String>& responses, bool auth=true) {
  if (auth && !isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  for (const auto& line : responses) {
    server.sendContent(line);
  }

  // End the response
  server.sendContent("");

  // Optional: close connection
  server.client().stop();
}

/* Summary of function: Generates and returns a salted 32-bit hardware ID. */
uint32_t generateDeviceToken() {
  uint32_t unique32bit = 0;

  #if defined(ESP32)
  // ESP32: Get 64-bit MAC and use the lower 32 bits
  uint64_t chipId = ESP.getEfuseMac();
  unique32bit = (uint32_t)(chipId & 0xFFFFFFFF);
  #elif defined(ESP8266)
  unique32bit = ESP.getChipId();
  #endif

  // Apply Grefur salt (0x5A4546)
  uint32_t salt = 0x5A4546;
  unique32bit ^= salt;

  return unique32bit;
}


/**
   @brief Error visualization controller with non-blocking LED patterns

   Each error type has a unique blink pattern for easy identification:

   Error Type               Blink Pattern              Total Duration
   ---------------------------------------------------------------
   INVALID_MEASUREMENT      2 quick blinks (300ms)     1.2s
   NO_INTERNET              4 medium blinks (500ms)    4.0s
   NO_BROKER                6 rapid blinks (200ms)     2.4s
   GENERAL_ERROR            3 slow blinks (700ms)      4.2s
   NETWORK_ERROR            3 slow blinks (700ms)      4.2s
   BROKER_ERROR             3 slow blinks (700ms)      4.2s
 * */

void debugPrint(const char* msg) {
  if (DEBUG_ERROR_VISUALIZATION) {
    Serial.println(msg);
  }
}

void debugPrintValue(const char* label, int value) {
  if (DEBUG_ERROR_VISUALIZATION) {
    Serial.print(label);
    Serial.print(": ");
    Serial.println(value);
  }
}

// ---------------- Error types ----------------
enum ErrorType {
  SYSTEM_OK,
  GENERAL_ERROR,
  INVALID_MEASUREMENT,
  NO_INTERNET,
  NO_BROKER,
  NETWORK_ERROR,
  BROKER_ERROR,
  WINK
};

// Forward declarations
void visualizationOfError(ErrorType error);
void updateErrorVisualization();
void debugPrint(const char* msg);
void debugPrintValue(const char* label, int value);

// ---------------- Feiltilstand ----------------
struct ErrorState {
  ErrorType currentError;
  uint8_t blinkCountRemaining;
  uint16_t blinkInterval;
  bool isActive;
  bool blinkState;
  unsigned long lastBlinkTime;
};

ErrorState errorState = {
                          GENERAL_ERROR,
                          0,
                          0,
                          false,
                          ONBOARD_LED_OFF,
                          0
                        };

// ---------------- Patterns structure ----------------
struct ErrorPattern {
  uint8_t blinkCount;   // sum of ON+OFF
  uint16_t interval;    // time per cycle in ms
};

const ErrorPattern errorPatterns[] = {
                                       {0, 0}, // SYSTEM_OK: 0 blinks means stay on
                                       {3, 700},   // GENERAL_ERROR
                                       {2, 300},   // INVALID_MEASUREMENT
                                       {4, 500},   // NO_INTERNET
                                       {6, 200},   // NO_BROKER
                                       {3, 700},   // NETWORK_ERROR
                                       {3, 700},   // BROKER_ERROR
                                       {10, 700},  // WINK
                                     };

// ---------------- Error visualization functions ----------------
void visualizationOfError(ErrorType error) {
  if (winkActive) return;

  // 1. If we are already blinking THIS error, don't restart it
  if (errorState.isActive && errorState.currentError == error) {
    return;
  }

  // 2. Handle the "System OK" state (Steady ON)
  if (error == SYSTEM_OK) {
    // Only set to steady ON if we aren't currently mid-blink for a real error
    if (!errorState.isActive) {
      errorState.currentError = SYSTEM_OK;
      digitalWrite(ONBOARD_LED, ONBOARD_LED_ON);
    }
    return;
  }

  // 3. Trigger a new Error Pattern
  errorState.currentError = error;
  errorState.blinkCountRemaining = errorPatterns[error].blinkCount * 2;
  errorState.blinkInterval = errorPatterns[error].interval;
  errorState.isActive = true;

  // Start with LED ON to ensure the flash is visible immediately
  errorState.blinkState = ONBOARD_LED_ON;
  errorState.lastBlinkTime = millis();
  digitalWrite(ONBOARD_LED, errorState.blinkState);

  debugPrint("New error pattern started");
}

void updateErrorVisualization() {
  if (!errorState.isActive) return;
  if (winkActive) return;

  if (millis() - errorState.lastBlinkTime >= errorState.blinkInterval) {
    errorState.lastBlinkTime = millis();

    errorState.blinkState = (errorState.blinkState == ONBOARD_LED_ON) ? ONBOARD_LED_OFF : ONBOARD_LED_ON;
    digitalWrite(ONBOARD_LED, errorState.blinkState);

    if (errorState.blinkState == ONBOARD_LED_ON) {
      errorState.blinkCountRemaining--;

      if (errorState.blinkCountRemaining <= 0) {
        errorState.isActive = false;
        digitalWrite(ONBOARD_LED, ONBOARD_LED_OFF);
        debugPrint("Pattern finished -> LED OFF");
      }
    }
  }
}





/**
   @brief MQTT module for IoT device communication
*/

/* ======================== */
/*      MQTT Connection     */
/* ======================== */

/* ======================== */
/*      Core Publishing     */
/* ======================== */

struct MqttValueCache {
  char topic[64];
  char payload[64];
  uint32_t lastUpdateTs;
  bool isActive = false;
};

class MqttManager {
  private:
    PubSubClient& mqttClient;
    int watchdogCounter = 0;
    char apName[32];  // Privat ram value
    int authRetries = 3; // Max attempts before locking
    MqttValueCache _valueCache[MAX_CALCULATIONS_ALLOWED];

    String getDeviceId(bool hex = true) {
    #if defined(ESP8266)
      return String(ESP.getChipId(), HEX);
    #elif defined(ESP32)
      uint64_t chipId = ESP.getEfuseMac();
      uint32_t shortId = (uint32_t)(chipId >> 32);
      return String(shortId, HEX);
    #endif
    }

    // List to store names of sensors that have already published metadata
    std::vector<String> publishedMetadata;

    void buildTopic(char* buffer, size_t len, const char* subtopic) {
      snprintf(buffer, len, "%s/%s/%s/%s", String(apName), config.mqtt.baseTopic, apName, subtopic);
    }

    bool isMetadataBlocked(const char* sensorName) {
      for (const String& name : publishedMetadata) {
        if (name.equals(sensorName)) return true;
      }
      return false;
    }

    bool authenticate() {
      if (WiFi.status() != WL_CONNECTED || authRetries <= 0) {
        #if DEBUG_MQTT_MANAGER
        if (authRetries <= 0) Serial.println("[AUTH] Maximum retries reached. Auth locked.");
        #endif
        return false;
      }

      HTTPClient http;

      String authUrl = String(GREFUR_BACKEND_URL) + "/api/rest/v1/devices/auth";
      #if DEBUG_MQTT_MANAGER
      Serial.println("[AUTH] Authenticating with Grefur Backend...");
      #endif

      #if defined(ESP8266)
      http.begin(netClient, authUrl);
      #elif defined(ESP32)
      http.begin(authUrl);
      #endif

      http.addHeader("Content-Type", "application/json");

      // Payload matcher 'DeviceAuthRequest' i C# backenden din
      StaticJsonDocument<128> authDoc;
      authDoc["ServiceValidationToken"] = config.device.productionToken;

      String requestBody;
      serializeJson(authDoc, requestBody);

      int httpResponseCode = http.POST(requestBody);

      if (httpResponseCode == 200) {
        String response = http.getString();
        StaticJsonDocument<512> resDoc;
        deserializeJson(resDoc, response);

        // Lagre mottatte credentials i config-objektet (RAM)
        // Her antar vi at auth-endepunktet returnerer alt vi trenger
        if (resDoc.containsKey("mqtt")) {
          JsonObject mqtt = resDoc["mqtt"];
          SaveConfigManager mgr(config);
          mgr.setSendHtml(false);

          // 1. Store credentials
          mgr.updateStringField(config.mqtt.username, sizeof(config.mqtt.username), mqtt["username"]);
          mgr.updateStringField(config.mqtt.password, sizeof(config.mqtt.password), mqtt["password"]);

          // Verify that broker details is existing in object
          if (mqtt.containsKey("brokerHost") && mqtt.containsKey("port")) {
            mgr.updateStringField(config.mqtt.brokerURL, sizeof(config.mqtt.brokerURL), mqtt["brokerHost"]);

            String portStr = mqtt["port"].as<String>();
            mgr.updateUint16Field(config.mqtt.port, portStr);

            // Use self.getApName as client ID if using grefur broker.
            mgr.updateStringField(config.mqtt.clientID, sizeof(config.mqtt.clientID), this->getApName());

            // Set flag for grefur ecosystem
            mgr.updateBoolField(config.mqtt.useGrefurBackend, true);

          }

          #if DEBUG_MQTT_MANAGER
          Serial.print("[DEBUG] MQTT Object: ");
          serializeJson(mqtt, Serial);
          Serial.println();
          Serial.printf("[AUTH] Success! Broker set to: %s:%d\n", config.mqtt.brokerURL, config.mqtt.port);
          #endif

          if (mgr.hasChanges()) {
            mgr.scheduleRestart();
          }

          authRetries = 3; // Reset retries

          mgr.finalize();
        }

        http.end();
        return true;
      } else if (httpResponseCode == 404 || httpResponseCode == 401) {
        // Enheten finnes ikke i portalen lenger
        #if DEBUG_MQTT_MANAGER
        Serial.println("[AUTH] Device not recognized by Grefur Portal.");
        #endif
        authRetries--;
      }

      http.end();
      return false;
    }

    void formatTopic(char* buffer, size_t len, const char* sensorName, const char* suffix = nullptr) {
      bool hasBase = (strlen(config.mqtt.baseTopic) > 0);
      bool hasSuffix = (suffix != nullptr && strlen(suffix) > 0);

      if (hasBase) {
        if (hasSuffix) {
          snprintf(buffer, len, "%s/%s/%s/%s", apName, config.mqtt.baseTopic, sensorName, suffix);
        } else {
          snprintf(buffer, len, "%s/%s/%s", apName, config.mqtt.baseTopic, sensorName);
        }
      } else {
        if (hasSuffix) {
          snprintf(buffer, len, "%s/%s/%s", apName, sensorName, suffix);
        } else {
          snprintf(buffer, len, "%s/%s", apName, sensorName);
        }
      }
    }

    /* Summary of function: Internal helper to publish with formatted topic */
    void publishFormatted(const char* sensorName, const char* payload, const char* suffix = nullptr, bool retain = false) {

      char topic[128];
      formatTopic(topic, sizeof(topic), sensorName, suffix);

      #if DEBUG_MQTT_MANAGER
      Serial.print("[MQTT] Publishing to ");
      Serial.print(topic);
      Serial.print(": ");
      Serial.println(payload);
      #endif

      mqttClient.publish(topic, payload, retain || config.mqtt.retain);
    }

    /* Summary of function: Internal helper for system info (ignores baseTopic) */
    void publishSystem(const char* subPath, const char* payload, bool retain = true) {
      char topic[128];
      // {apName}/{subPath}
      snprintf(topic, sizeof(topic), "%s/%s", apName, subPath);

      #if DEBUG_MQTT_MANAGER
      Serial.printf("[MQTT SYSTEM] %s -> %s\n", topic, payload);
      #endif

      mqttClient.publish(topic, payload, retain);
    }



  public:
    MqttManager(PubSubClient& client) : mqttClient(client) {
      // Inisiliase apName in constructor
      String generatedName = generateAPName();
      strncpy(apName, generatedName.c_str(), sizeof(apName));
      apName[sizeof(apName) - 1] = '\0';


    }

    // ========================
    // Core Publishing (alltid UTEN apName)
    // ========================
    void publishNestedValue(const char* subtopic, const char* payload, bool retain = false) {
      char topic[128];

      if (strlen(config.mqtt.baseTopic) > 0) {
        snprintf(topic, sizeof(topic), "%s/%s/%s", apName, config.mqtt.baseTopic, subtopic);
      } else {
        snprintf(topic, sizeof(topic), "%s/%s", apName, subtopic);
      }

      if (DEBUG_MQTT_MANAGER) {
        Serial.print("[DEBUG] publishNestedValue: ");
        Serial.println(topic);
      }

      if (retain){
        mqttClient.publish(topic, payload, retain);
      } else {
        mqttClient.publish(topic, payload, config.mqtt.retain);
      }
    }

    void publishFlatValue(const char* topic, const char* payload, bool retain = false) {
      #if DEBUG_MQTT_MANAGER
      Serial.print("[DEBUG] publishFlatValue: ");
      Serial.print(topic);
      Serial.print(" = ");
      Serial.println(payload);
      #endif

      if (retain){
        mqttClient.publish(topic, payload, retain);
      } else {
        mqttClient.publish(topic, payload, config.mqtt.retain);
      }
    }


    // ========================
    // Data Publications
    // ========================
    /* Summary of function: Publishes measured sensor value with optional unit and metadata */
    void publishMeasuredValue(float value, const char* sensorName, const char* unit, signed int valueType = -1) {
      if (config.mqtt.useNestedJson) {
        char payload[128];
        if (unit && strlen(unit) > 0) {
          snprintf(payload, sizeof(payload), "{\"value\":%.2f,\"unit\":\"%s\",\"type\":%d}", value, unit, valueType);
        } else {
          snprintf(payload, sizeof(payload), "{\"value\":%.2f}", value);
        }
        publishFormatted(sensorName, payload);
      } else {
        char valueStr[16];
        dtostrf(value, 4, 2, valueStr);
        publishFormatted(sensorName, valueStr, "value");

        if (!isMetadataBlocked(sensorName)) {
          char typePayload[8];
          itoa(valueType, typePayload, 10);
          publishFormatted(sensorName, typePayload, "type", true);

          if (unit && strlen(unit) > 0) {
            publishFormatted(sensorName, unit, "unit", true);
          }
          publishedMetadata.push_back(String(sensorName));
        }
      }
    }

    // ========================
    // Subscription Logic
    // ========================

    /* Summary of function: Subscribes to a specific sensor or system topic with flexible nesting */
    void subscribeToTopic(const char* topic, bool isNested, const char* topicEnd = "set", const char* nestedKey = nullptr) {
      char resolvedTopic[128];

      if (isNested) {
        // Logic for nested: {apName}/{baseTopic}/{topic}
        // formatTopic handles the internal concatenation logic
        formatTopic(resolvedTopic, sizeof(resolvedTopic), topic);
      } else {
        // Logic for flat: Use the provided topic directly as the full path
        strncpy(resolvedTopic, topic, sizeof(resolvedTopic));
        resolvedTopic[sizeof(resolvedTopic) - 1] = '\0';
      }

      #if DEBUG_MQTT_MANAGER
      Serial.print("[MQTT] Subscribing to: ");
      Serial.println(resolvedTopic);
      #endif

      mqttClient.subscribe(resolvedTopic);
    }

    /* Summary of function: Unsubscribes from a specific sensor or system topic using the same resolution logic as subscribe */
    void unsubscribeFromTopic(const char* topic, bool isNested) {
      char resolvedTopic[128];

      if (isNested) {
        // Resolve the Grefur-specific nested path: {apName}/{baseTopic}/{topic}
        formatTopic(resolvedTopic, sizeof(resolvedTopic), topic);
      } else {
        // Resolve the flat path: Use the provided topic directly
        strncpy(resolvedTopic, topic, sizeof(resolvedTopic));
        resolvedTopic[sizeof(resolvedTopic) - 1] = '\0';
      }

      #if DEBUG_MQTT_MANAGER
      Serial.print("[MQTT] Unsubscribing from: ");
      Serial.println(resolvedTopic);
      #endif

      // Instruct the PubSubClient to send the MQTT UNSUBSCRIBE packet
      mqttClient.unsubscribe(resolvedTopic);
    }

    /* Summary of function: Saves incoming topic, payload, and current timestamp into internal RAM cache */
    void handleIncomingPayload(const char* topic, const char* message) {
      int firstFreeSlot = -1;

      #if DEBUG_MQTT_MANAGER
      Serial.print("[MQTT] Incoming dataset to cache for topic: ");
      Serial.println(topic);
      #endif

      for (int i = 0; i < MAX_CALCULATIONS_ALLOWED; i++) {
        // Check if topic already exists in cache
        if (_valueCache[i].isActive && strcmp(_valueCache[i].topic, topic) == 0) {
          strncpy(_valueCache[i].payload, message, sizeof(_valueCache[i].payload) - 1);
          _valueCache[i].lastUpdateTs = millis();
          return;
        }
        if (firstFreeSlot == -1 && !_valueCache[i].isActive) {
          firstFreeSlot = i;
        }
      }

      // If new topic, use the first free slot
      if (firstFreeSlot != -1) {
        _valueCache[firstFreeSlot].isActive = true;
        strncpy(_valueCache[firstFreeSlot].topic, topic, sizeof(_valueCache[firstFreeSlot].topic) - 1);
        strncpy(_valueCache[firstFreeSlot].payload, message, sizeof(_valueCache[firstFreeSlot].payload) - 1);
        _valueCache[firstFreeSlot].lastUpdateTs = millis();
      }
    }

    /* Summary of function: Searches cache by topic and returns payload and timestamp via pointers */
    bool requestExternalValueByTopic(const char* topic, String& outPayload, uint32_t& outDateTime) {
      #if DEBUG_MQTT_MANAGER
      Serial.print("[MQTT] Incoming request for cache by topic: ");
      Serial.println(topic);
      #endif

      for (int i = 0; i < MAX_CALCULATIONS_ALLOWED; i++) {
        if (_valueCache[i].isActive && strcmp(_valueCache[i].topic, topic) == 0) {
          outPayload = String(_valueCache[i].payload);
          outDateTime = _valueCache[i].lastUpdateTs;
          return true;
        }
      }
      return false; // Topic not found in cache
    }





    /* Summary of function: Utility to subscribe to common device commands */
    void subscribeToCommands() {
      // Example: Subscribing to {apName}/commands/execute
      subscribeToTopic("commands", true, "execute");
    }




    void clearBlockedSensors() {
      publishedMetadata.clear();
        #if DEBUG_MQTT_MANAGER
      Serial.println("[DEBUG] Metadata block-list cleared.");
        #endif
    }

    void publishIP() {
      String ip = WiFi.localIP().toString();
      #if DEBUG_MQTT_MANAGER
      Serial.print("[DEBUG] IP published: ");
      Serial.println(ip);
      #endif

      if (config.mqtt.useNestedJson) {
        char payload[64];
        snprintf(payload, sizeof(payload), "{\"value\":\"%s\"}", ip.c_str());
        publishFormatted("info/ip", payload);
      } else {
        publishFormatted("info/ip", ip.c_str());
      }
    }

    /* Summary of function: Publishes current device status (online/offline) and RSSI */
    void publishStatus(const char* status) {
      char topicBuffer[128];
      snprintf(topicBuffer, sizeof(topicBuffer), "%s/status", apName);

      if (config.mqtt.useNestedJson) {
        char payload[256];
        snprintf(payload, sizeof(payload), "{\"value\":\"%s\",\"rssi\":%d,\"version\":\"%s\"}",
                 status, WiFi.RSSI(), config.header.version);
        mqttClient.publish(topicBuffer, payload, true);
      } else {
        mqttClient.publish(topicBuffer, status, true);
        if (config.operation.publishRSSI) {
          char rssiStr[16];
          itoa(WiFi.RSSI(), rssiStr, 10);
          char rssiTopic[128];
          snprintf(rssiTopic, sizeof(rssiTopic), "%s/status/rssi", apName);
          mqttClient.publish(rssiTopic, rssiStr);
        }
      }
    }


    /* Summary of function: Publishes comprehensive device information */
    void publishDeviceInfo() {
      if (config.mqtt.useNestedJson) {
        StaticJsonDocument<512> doc;
        doc["deviceId"] = getDeviceId(false);
        doc["deviceName"] = config.device.deviceName;
        doc["location"] = config.device.location;
        doc["firmware"] = config.header.version;
        doc["ip"] = WiFi.localIP().toString();
        doc["mac"] = WiFi.macAddress();
        doc["baseTopic"] = String(config.mqtt.baseTopic);

        String payload;
        serializeJson(doc, payload);
        publishSystem("info", payload.c_str());
      } else {
        // Flat mode: Alt publiseres direkte under {apName}/info/...
        publishSystem("info/deviceId", getDeviceId(true).c_str());
        publishSystem("info/deviceName", config.device.deviceName);
        publishSystem("info/location", config.device.location);
        publishSystem("info/ip", WiFi.localIP().toString().c_str());
        publishSystem("info/baseTopic", config.mqtt.baseTopic);
        publishSystem("info/firmware", config.header.version);
        publishSystem("info/mac", WiFi.macAddress().c_str());
      }
    }

    /* Summary of function: Publishes a watchdog counter to verify live connection */
    void publishWatchdog() {
      watchdogCounter++;
      if (watchdogCounter > 255) watchdogCounter = 1;

      char payload[64];
      if (config.mqtt.useNestedJson) {
        snprintf(payload, sizeof(payload), "{\"value\":%d}", watchdogCounter);
        publishFormatted("watchdog", payload);
      } else {
        itoa(watchdogCounter, payload, 10);
        publishFormatted("watchdog", payload);
      }
    }

    // ========================
    // MQTT Connection
    // ========================
    /* Summary of function: Handles MQTT connection with lazy authentication.
                                                                                                                                                                                                                                                                                                                                                               Only requests new credentials if current ones are missing or rejected. */
    bool reconnect() {
      if (mqttClient.connected()) return true;

      char lastWillTopic[128];
      snprintf(lastWillTopic, sizeof(lastWillTopic), "%s/status", apName);

      if (mqttClient.connect(
            config.mqtt.clientID,
            config.mqtt.username,
            config.mqtt.password,
            lastWillTopic,
            config.mqtt.qos,
            true,
            "offline")) {

        #if DEBUG_MQTT_MANAGER
        Serial.println("[MQTT] Connected successfully!");
        #endif

        if (config.operation.publishStatus) publishStatus("online");
        if (config.operation.publishIP) publishIP();
        clearBlockedSensors();

        if (config.operation.publishDeviceInfo) {
          publishDeviceInfo();
        }



        return true;

      } else {
        int state = mqttClient.state();

        #if DEBUG_MQTT_MANAGER
        Serial.printf("Use Grefur Backend Service: %s\n", config.mqtt.useGrefurBackend ? "Yes" : "No");
        #endif

        if ((state == 5 || state == 4) && config.mqtt.useGrefurBackend || (state == -2) && config.mqtt.useGrefurBackend) {
          #if DEBUG_MQTT_MANAGER
          Serial.println("[MQTT] Connection rejected");
          #endif

          // If part of the ecosystem, it will try to ask for new credentials
          // This will eventually work next retry.
          if (config.mqtt.useGrefurBackend){
             #if DEBUG_MQTT_MANAGER
            Serial.println("[MQTT] Fetching fresh credentials...");
             #endif
            authenticate();
            return false;
          }
        }
        else if (state == -2) {
      #if DEBUG_MQTT_MANAGER
          Serial.println("[MQTT] Broker unreachable. Retrying later.");
      #endif
          visualizationOfError(NO_BROKER);
        } else {
      #if DEBUG_MQTT_MANAGER
          Serial.printf("[MQTT] Failed, rc=%d\n", state);
      #endif
          visualizationOfError(BROKER_ERROR);
        }
        return false;
      }
    }
    const char* getApName() {
      return apName;
    }
};


class MqttManager;
extern MqttManager mqtt;


/**
   @brief Sensor Manager - Handles sensor configuration and measurements

   Provides functionality for:
   - Adding/removing sensors of different types (NTC, Analog, Digital)
   - Storing/loading sensor configurations to/from EEPROM
   - Taking measurements with automatic scaling and validation
   - Generating UI metadata for sensor configuration forms

   Supported sensor types:
   1. NTC Thermistors (temperature)
   2. Analog sensors (voltage/current)
   3. Digital sensors (on/off)

   All configurations are stored in the EEPROM structure and maintain
   checksum validation with the existing system.

   Usage:
   1. Initialize with global config reference
   2. Call addSensor() with configuration
   3. Use readSensor() to get measurements
   4. Access configuration via getSensorConfig()
*/

#define DEBUG_SENSOR_MANAGER false

class SensorManager {
  private:
    EEPROMLayout& _config;
    bool _changed;
  public:
    SensorManager(EEPROMLayout& config) : _config(config), _changed(false) {}
    float _currentReadings[MAX_SENSORS_ALLOWED] = {}; // Fresh measurments, not published

    // Get available sensor types
    String retrieveSensorTypes() {
      DynamicJsonDocument doc(256);
      JsonArray types = doc.createNestedArray("types");

      types.add("NTC");
      types.add("Analog");
      types.add("Digital");

      String output;
      serializeJson(doc, output);
      return output;
    }

    // Get fields for a specific sensor type
    String retrieveSensorFieldsAsJson(int8_t type) {
      DynamicJsonDocument doc(512);
      JsonObject fields = doc.createNestedObject("fields");
      JsonObject labels = doc.createNestedObject("labels");
      JsonObject units = doc.createNestedObject("units");
      JsonObject placeholders = doc.createNestedObject("placeholders");

      switch (type) {
        case SENSOR_NTC:
          doc["type"] = "NTC";

          // Series Resistor
          fields["seriesResistor"] = "number";
          labels["seriesResistor"] = "Series Resistor";
          units["seriesResistor"] = "Ω";
          placeholders["seriesResistor"] = "e.g., 10000";

          // Nominal Resistance
          fields["nominalResistance"] = "number";
          labels["nominalResistance"] = "Nominal Resistance";
          units["nominalResistance"] = "Ω";
          placeholders["nominalResistance"] = "e.g., 10000";

          // Nominal Temperature
          fields["nominalTemp"] = "number";
          labels["nominalTemp"] = "Nominal Temperature";
          units["nominalTemp"] = "°C";
          placeholders["nominalTemp"] = "e.g., 25.0";

          // B Coefficient
          fields["bCoefficient"] = "number";
          labels["bCoefficient"] = "B Coefficient";
          units["bCoefficient"] = "";
          placeholders["bCoefficient"] = "e.g., 3950";

          fields["analogPin"] = "number";
          labels["analogPin"] = "Connected Input";
          units["analogPin"] = "";
          placeholders["analogPin"] = "e.g., A0";

          fields["unit"] = "text";
          labels["unit"] = "Temperature unit";
          units["unit"] = "";
          placeholders["unit"] = "e.g., °C";

          break;

        case SENSOR_ANALOG:
          doc["type"] = "Analog";

          // Analog Pin
          fields["analogPin"] = "number";
          labels["analogPin"] = "Analog Pin";
          units["analogPin"] = "";
          placeholders["analogPin"] = "e.g., A0";

          // Input Minimum
          fields["inputMin"] = "number";
          labels["inputMin"] = "Min Voltage";
          units["inputMin"] = "V";
          placeholders["inputMin"] = "e.g., 0.0";

          // Input Maximum
          fields["inputMax"] = "number";
          labels["inputMax"] = "Max Voltage";
          units["inputMax"] = "V";
          placeholders["inputMax"] = "e.g., 3.3";

          // Scale Minimum
          fields["scaleMin"] = "number";
          labels["scaleMin"] = "Scale Min";
          units["scaleMin"] = "";
          placeholders["scaleMin"] = "e.g., 0";

          // Scale Maximum
          fields["scaleMax"] = "number";
          labels["scaleMax"] = "Scale Max";
          units["scaleMax"] = "";
          placeholders["scaleMax"] = "e.g., 100";

          fields["scaleUnit"] = "text";
          labels["scaleUnit"] = "Scaled unit";
          units["scaleUnit"] = "";
          placeholders["scaleUnit"] = "e.g., kPa";

          break;

        case SENSOR_DIGITAL:
          doc["type"] = "Digital";

          // Digital Pin
          fields["digitalPin"] = "number";
          labels["digitalPin"] = "Digital Pin";
          units["digitalPin"] = "";
          placeholders["digitalPin"] = "e.g., 5";

          // Inverted Logic
          fields["inverted"] = "boolean";
          labels["inverted"] = "Inverted Logic";
          units["inverted"] = "";
          placeholders["inverted"] = "";
          break;
      }

      String output;
      serializeJson(doc, output);
      return output;
    }


    // Get all configured sensors
    String retrieveAllSensors(bool returnEverySensor = false) {
      DynamicJsonDocument doc(1024);
      JsonArray sensors = doc.createNestedArray("sensors");

      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        // A sensor is considered configured only if:
        // - type is NOT -1 (means deleted/unconfigured)
        // AND
        // - type != 0, or (type == 0 and it has a name)
        int sensorType = int(_config.sensors[i].type);

        bool isConfigured = (sensorType >= 0 && strlen(_config.sensors[i].name) > 0);

        if (returnEverySensor || isConfigured) {
          JsonObject sensor = sensors.createNestedObject();
          sensor["id"] = i;
          sensor["type"] = static_cast<int8_t>(_config.sensors[i].type);
          sensor["name"] = _config.sensors[i].name;
          sensor["configured"] = isConfigured;

          // Add type-specific details for configured sensors
          if (isConfigured) {
            switch (_config.sensors[i].type) {
              case SENSOR_NTC:
                sensor["details"] = String(_config.sensors[i].ntc.nominalResistance) + "Ω";
                break;
              case SENSOR_ANALOG:
                sensor["details"] = "Pin A" + String(_config.sensors[i].analog.analogPin);
                break;
              case SENSOR_DIGITAL:
                sensor["details"] = "GPIO" + String(_config.sensors[i].digital.digitalPin);
                break;
              default:
                if (_config.sensors[i].type == 0) {
                  sensor["details"] = "Legacy NTC";  // Handle case where NTC is type 0
                }
            }
          } else {
            sensor["status"] = "unconfigured";
          }
        }
      }

      String output;
      serializeJson(doc, output);
      return output;
    }

    // Add a new sensor
    int addSensor(const SensorConfig& newSensor) {
      Serial.println("\nSearching for empty slots...");

      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        Serial.printf("Checking for slot %d: type=%d, name='%s'\n",
                      i, _config.sensors[i].type, _config.sensors[i].name);

        // Extended check for empty slots
        if (_config.sensors[i].type == SENSOR_UNCONFIGURED ||
            (_config.sensors[i].type == 0 && strlen(_config.sensors[i].name) == 0)) {

          // Full copy using memcpy to prevent partful copies
          memcpy(&_config.sensors[i], &newSensor, sizeof(SensorConfig));
          _changed = true;

          if (DEBUG) {
            Serial.println("Added new sensor:");
            Serial.printf("- Type: %d\n", _config.sensors[i].type);
            Serial.printf("- Navn: '%s'\n", _config.sensors[i].name);

            if (_config.sensors[i].type == 0) { // NTC-sensor
              Serial.printf("- Enhet: '%s'\n", _config.sensors[i].ntc.unit);
            }
          }
          return i;
        }
      }

      if (DEBUG) {
        Serial.println("No empty space found - expected empty slot");
      }

      printAllSensorSlots();
      return -1;
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         * Updates the configuration for an existing sensor given its ID (index).
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         * * @param sensorId The ID (index) of the sensor slot to update.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         * @param updatedSensor The new SensorConfig object to replace the existing one.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         * @return sensorId if the update was successful, otherwise -1.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         */
    int updateSensor(int sensorId, const SensorConfig& updatedSensor) {
      Serial.println("\nSearching for slot to update...");

      // 1. Validate the sensorId against the maximum allowed limit.
      if (sensorId < 0 || sensorId >= MAX_SENSORS_ALLOWED) {
        if (DEBUG) {
          Serial.printf("Error: Invalid sensorId %d. Max allowed is %d\n", sensorId, MAX_SENSORS_ALLOWED - 1);
        }
        return -1;
      }

      // 2. Optional but recommended check: ensure the slot is already configured.
      // We check for SENSOR_UNCONFIGURED (likely an enum value for 'empty')
      // or a common zero-value empty state (type=0 and empty name).
      if (_config.sensors[sensorId].type == SENSOR_UNCONFIGURED ||
          (_config.sensors[sensorId].type == 0 && strlen(_config.sensors[sensorId].name) == 0)) {
        if (DEBUG) {
          Serial.printf("Error: Slot %d is unconfigured (type=%d, name='%s'). Use addSensor instead of updateSensor.\n",
                        sensorId, _config.sensors[sensorId].type, _config.sensors[sensorId].name);
        }
        return -1;
      }

      // 3. Perform a full byte-by-byte copy of the new configuration over the old one.
      memcpy(&_config.sensors[sensorId], &updatedSensor, sizeof(SensorConfig));

      // 4. Set the changed flag to ensure the configuration is saved (e.g., to EEPROM/Flash) later.
      _changed = true;

      if (DEBUG) {
        Serial.println("Updated existing sensor:");
        Serial.printf("- Slot: %d\n", sensorId);
        Serial.printf("- Type: %d\n", _config.sensors[sensorId].type);
        Serial.printf("- Name: '%s'\n", _config.sensors[sensorId].name);

        if (_config.sensors[sensorId].type == 0) { // Assuming type 0 is NTC-sensor from the example logic
          Serial.printf("- Unit: '%s'\n", _config.sensors[sensorId].ntc.unit);
        }
      }

      return sensorId;
    }

    // Remove a sensor
    bool removeSensor(int id) {
      if (id < 0 || id >= MAX_SENSORS_ALLOWED) {
        Serial.printf("Invalid sensor ID %d\n", id);
        return false;
      }

      if (DEBUG) {
        Serial.printf("Deleting sensor %d - Before:\n", id);
        Serial.printf("Type: %d, Name: '%s'\n",
                      _config.sensors[id].type,
                      _config.sensors[id].name);
      }

      // Full reset
      SensorConfig blankConfig = {};
      blankConfig.type = SENSOR_UNCONFIGURED; // Explicitly set to -1
      memset(blankConfig.name, 0, sizeof(blankConfig.name));
      _config.sensors[id] = blankConfig;
      _changed = true;

      if (DEBUG) {
        Serial.printf("After deletion:\n");
        Serial.printf("Type: %d, Name: '%s'\n",
                      _config.sensors[id].type,
                      _config.sensors[id].name);
      }

      return true;
    }

    bool isSensorOfIndexConfigured(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return false;
      }
      // Assuming a type >= 0 (or a specific enum like SENSOR_UNCONFIGURED)
      // defines a valid configuration.
      return _config.sensors[index].type >= 0;
    }

    // Check if any sensor is configured
    bool isSensorConfigured() const {
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        if (_config.sensors[i].type < 0) {
          return true;
        }
      }
      return false;
    }

    // Read sensor value based on configured type
    float readSensor(int sensorIndex) {
      if (sensorIndex < 0 || sensorIndex >= MAX_SENSORS_ALLOWED) {
        return NAN;
      }

      switch (_config.sensors[sensorIndex].type) {
        case SENSOR_NTC:
          return readTemperature(sensorIndex);
        case SENSOR_ANALOG:
          return readVoltage(sensorIndex);
        case SENSOR_DIGITAL:
          return readDigital(sensorIndex) ? 1.0f : 0.0f;
        default:
          return NAN;
      }
    }


    // Get current sensor configuration
    const SensorConfig& getSensorConfig(int index) const {
      if (index >= 0 && index < MAX_SENSORS_ALLOWED) {
        return _config.sensors[index];
      }
      static SensorConfig emptyConfig = {0};
      return emptyConfig;
    }

    // Check if there are unsaved changes
    bool hasChanges() const {
      return _changed;
    }

    // Save changes to EEPROM
    bool saveChanges() {
      if (_changed) {
        Serial.println("Saving sensor changes to configuration...");
        _changed = false; // Reset first to prevent recursion

        // Use the global saveConfig() function
        if (saveConfig()) {
          Serial.println("Sensor changes successfully saved");
          return true;
        }

        Serial.println("Failed to save sensor changes");
        _changed = true; // Restore changed flag if save failed
        return false;
      }
      return true; // No changes to save
    }

    int getConfiguredSensorCount() const {
      int count = 0;
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        if (_config.sensors[i].type >= 0) count++;
      }
      return count;
    }

    void printSensorStates() {
      Serial.println("\nCurrent Sensor States:");
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        Serial.printf("Sensor %d: type=%d, name='%s'\n",
                      i, config.sensors[i].type, config.sensors[i].name);
      }
    }

    // Use when reset all slots
    void initializeSensorSlots() {
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        _config.sensors[i].type = SENSOR_UNCONFIGURED; // Setter til -1
        memset(_config.sensors[i].name, 0, sizeof(_config.sensors[i].name));
      }
      _changed = true;
    }

    void printAllSensorSlots() {
      Serial.println("\nAlle sensorplasser:");
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        Serial.printf("[%d] type=%d, name='%s'",
                      i, _config.sensors[i].type, _config.sensors[i].name);

        if (_config.sensors[i].type == 0) { // NTC
          Serial.printf(", unit='%s'", _config.sensors[i].ntc.unit);
        }
        Serial.println();
      }
    }

    String getSensorUnit(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return "";  // Ugyldig indeks
      }

      switch (_config.sensors[index].type) {
        case SENSOR_NTC:
          // Enheten ligger i ntc.unit (f.eks "°C")
          return String(_config.sensors[index].ntc.unit);

        case SENSOR_ANALOG:
          // Enheten er i analog.scaleUnit
          return String(_config.sensors[index].analog.scaleUnit);

        case SENSOR_DIGITAL:
          // Digital sensorer har som regel ikke enhet, returner tom streng
          return "";

        default:
          return "";
      }
    }

    String getSensorName(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return "";  // Ugyldig indeks
      }

      return _config.sensors[index].name;
    }

    bool sameValueLastTime(int index, float value) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return false;  // Invalid index
      }

      return _config.sensors[index].lastMeasurement == value;
    }

    bool isPublishOnlyOnChange(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) {
        return false;
      }
      return _config.sensors[index].publishPolicy.trigger == PUBLISH_ON_CHANGE_OF_THRESHOLD;
    }


    void setLastMeasurement(int index, float value) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return;
      _config.sensors[index].lastMeasurement = value;
      #if DEBUG_SENSOR_MANAGER
      Serial.printf("Last measurement value %.2f was set for sensor with index %d\n", value, index);
      #endif
    }

    float getLastMeasurement(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return NAN;
      float lastMeasurement = _config.sensors[index].lastMeasurement;
      #if DEBUG_SENSOR_MANAGER
      Serial.printf("Id: %d - Last measurement %.2f\n", index, lastMeasurement);
      #endif
      return lastMeasurement;
    }

    void setCurrentReading(int index, float value) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return;
      _currentReadings[index] = value;
    }

    float getCurrentReading(int index) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return NAN;
      return _currentReadings[index];
    }

    void setLastUpdateTs(int index, uint32_t ts) {
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return;
      _config.sensors[index].lastUpdateTs = ts;
    }

    bool isBooleanSignal(int index){
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return NAN;

      if (_config.sensors[index].type == SENSOR_DIGITAL){
        return true;
      } else {
        return false;
      }
    }

    signed int getSensorType(int index){
      if (index < 0 || index >= MAX_SENSORS_ALLOWED) return NAN;

      return _config.sensors[index].type;
    }

    float retrieveLastValueByName(const String& name) const {
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        const SensorConfig& s = _config.sensors[i];
        if (s.type >= 0 && String(s.name) == name) {
          return _currentReadings[i];  // ← bruk currentReading istedenfor
        }
      }
      return NAN;
    }




  private:
    // DEBUG_MUX_FUNCTIONALITY

    int getAdcMaxValue(){
      // Determine ADC resolution
      #if defined(ESP8266)
      return 1023;
      #elif defined(ESP32)
      return 4095;
      #else
      return 4095;
      #endif
    }

    float analogReadAvg(int pin, int samples = 10) {
      float sum = 0;
      for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
        delay(2);
      }
      return sum / samples;
    }

    /* Based on type, will the analogReadMux work in muliple ways
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * ESP6822: Defines mux channel to read using the analog pin from 0-3
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *          Set bit 0 and 1 in correct state and output them to given MUX PIN defined in header
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *          Stabilies the hardware
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *          Make the read
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *          Set both MUX pins low
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *          Return measured value as float
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * ESP32:   Defines analog pin to read based on type
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *          Read given pin 10 times and use avarage
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     *          Return value as float
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    float analogReadMux(int sensorIndex) {

      // Use the ternary operator to assign the correct pin number directly to analogPin.
      int analogPin = (_config.sensors[sensorIndex].type == SENSOR_NTC)
                      ? _config.sensors[sensorIndex].ntc.analogPin
                      : _config.sensors[sensorIndex].analog.analogPin; // Assumes SENSOR_ANALOG if not NTC


      #if defined(ESP8266)
      if (DEBUG && DEBUG_MUX_FUNCTIONALITY) {
        Serial.println("Read channel " + String(sensorIndex) + " using multiplexer...");
      }

      if (DEBUG && DEBUG_MUX_FUNCTIONALITY) {
        Serial.println("Mux index: " + String(analogPin));
      }

      bool MUX_A_STATE = (analogPin & 0b01) != 0; // Check if bit 0 is true
      bool MUX_B_STATE = (analogPin & 0b10) != 0; // Check if bit 1 is true

      // Set mux channel for correct analog read
      digitalWrite(MUX_A, MUX_A_STATE);  // MSB
      digitalWrite(MUX_B, MUX_B_STATE);  // LSB

      if (DEBUG && DEBUG_MUX_FUNCTIONALITY) {
        Serial.println("PIN A: " + String(MUX_A_STATE) + " | PIN B: " + String(MUX_B_STATE));
      }

      delay(7); // stabilisation

      int readValue = analogRead(A0); // mux output read

      digitalWrite(MUX_A, LOW);
      digitalWrite(MUX_B, LOW);

      return (float)readValue;
      #elif defined(ESP32)

      int readValue = analogReadAvg(analogPin, 10);
      #if DEBUG_SENSOR_MANAGER
      char buf[64];
      snprintf(buf, sizeof(buf), "Max value: %d - Read value: %d", getAdcMaxValue(), readValue);
      Serial.println(buf);
      #endif

      return (float)readValue;


      #endif

    }





    float fmap(float x, float inMin, float inMax, float outMin, float outMax) {
      return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }


    float readTemperature(int index) {
      const float adcMax = getAdcMaxValue();

      float adcValue = analogReadMux(index);

      // Used for
      //float R = _config.sensors[index].ntc.seriesResistor / ((adcMax / adcValue) - 1.0);

      float R = _config.sensors[index].ntc.seriesResistor * adcValue / (adcMax - adcValue);

      return 1.0 / (log(R / _config.sensors[index].ntc.nominalResistance) /
                    _config.sensors[index].ntc.bCoefficient +
                    1.0 / (_config.sensors[index].ntc.nominalTemp + 273.15)) - 273.15;
    }



    float readVoltage(int index) {
      float raw = (float)analogReadMux(index);

      #if DEBUG_SENSOR_MANAGER
      Serial.print("Measured voltage value:");
      Serial.println(raw);
      #endif

      return fmap(raw,
                  _config.sensors[index].analog.inputMin,
                  _config.sensors[index].analog.inputMax,
                  _config.sensors[index].analog.scaleMin,
                  _config.sensors[index].analog.scaleMax);
    }

    bool readDigital(int index) {
      int pin = _config.sensors[index].digital.digitalPin;

      #if defined(ESP8266)
      bool val = digitalRead(pin);
      return _config.sensors[index].digital.inverted ? !val : val;
      #elif defined(ESP32)

      //String pinStr = String("D" + String(pin));
      bool val = digitalRead(pin);
      return _config.sensors[index].digital.inverted ? !val : val;
       #endif
    }
};

SensorManager sensorManager(config);  // Global instance


// Middelware to take measurements from all configured sensors
void takeMeasurements(float outValues[]) {
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    outValues[i] = NAN; // default verdi
    if (i < (sizeof(config.sensors) / sizeof(config.sensors[0]))) {
      if (config.sensors[i].type != SENSOR_UNCONFIGURED) {
        outValues[i] = sensorManager.readSensor(i);
      }
    }
  }
}



/*
 *     MANAGER FOR EXPRESSION
 *
 */




class ExpressionManager {
  private:
    CalculationConfig* _configs;
    int _numConfigs;

    struct RuntimeState {
      float lastValue;
      float lastTriggerValue;
      // PID State persistence
      float integralSum;
      float lastError;
      uint32_t lastProcessTime;

      // State for DELAY function
      float delayTargetValue;
      uint32_t delayStartTime;

      bool delayInitialized;
    };

    RuntimeState _runtime[MAX_CALCULATIONS_ALLOWED];

  public:
    ExpressionManager(CalculationConfig* configs, int numConfigs)
      : _configs(configs), _numConfigs(numConfigs)
    {
      for (int i = 0; i < _numConfigs; i++) {
        resetRuntimeState(i);
      }
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Resets the runtime variables for a specific slot.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Essential for clearing PID history when configs change.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    void resetRuntimeState(int index) {
      if (index >= 0 && index < MAX_CALCULATIONS_ALLOWED) {
        _runtime[index].lastValue = NAN;
        _runtime[index].lastTriggerValue = NAN;
        _runtime[index].integralSum = 0.0f;
        _runtime[index].lastError = 0.0f;
        _runtime[index].lastProcessTime = 0;
        _runtime[index].delayTargetValue = NAN;
        _runtime[index].delayStartTime = 0;
        _runtime[index].delayInitialized = false;
      }
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Returns total number of active calculation slots.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    int getConfiguredCalculationCount() {
      int count = 0;
      for (int i = 0; i < _numConfigs; i++) {
        // Include EXTERNAL in the count
        if (_configs[i].type == CALC_EXPRESSION ||
            _configs[i].type == CALC_CONTINUOUS ||
            _configs[i].type == CALC_EXTERNAL) {
          count++;
        }
      }
      return count;
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Iterates through all configurations and processes logic based on type.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    /**
                                                                                                                                                                                                                                                                                                                     * Summary of function: Iterates through all configurations and processes logic based on type.
                                                                                                                                                                                                                                                                                                                     * Updated to use millis() for timing and handle CALC_EXTERNAL timeout logic.
                                                                                                                                                                                                                                                                                                                     */
    /* Summary of function: Main evaluation loop. Iterates once through configs and routes to appropriate handler. */
    void evaluateAll(uint32_t deltaMs) {
      for (int i = 0; i < _numConfigs; i++) {
        switch (_configs[i].type) {
          case CALC_EXTERNAL:
            // Pull from cache and check timeout in one go
            handleExternalMessages(i);
            break;

          case CALC_EXPRESSION:
            evaluateExpression(i);
            break;

          case CALC_CONTINUOUS:
            evaluateContinuous(i, deltaMs);
            break;

          default:
            break;
        }
      }
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Checks if the expression syntax is valid before saving.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    bool validateExpression(const String& expr, String& errorMessage) {
      String s = expr;
      s.trim();
      if (s.length() == 0) {
        errorMessage = "Expression is empty";
        return false;
      }
      int questions = 0, colons = 0, brackets = 0;
      for (int i = 0; i < s.length(); i++) {
        if (s[i] == '?') questions++;
        else if (s[i] == ':') colons++;
        else if (s[i] == '(') brackets++;
        else if (s[i] == ')') brackets--;
        if (brackets < 0) {
          errorMessage = "Mismatched parentheses";
          return false;
        }
      }
      return (questions == colons && brackets == 0);
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Adds a new calculation config to the first available slot.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    /* Summary of function: Adds a new calculation config. External bindings are placed from index 0 upwards,
                                                                                                                                                                                                                                                   while expressions/continuous types are placed from the last available slot downwards. */
    int addCalculation(const CalculationConfig& newCalc, MqttManager& mqtt) {
      if (newCalc.type == CALC_EXTERNAL) {
        // 1. Logic for External Bindings: Start from 0 and find first empty slot
        for (int i = 0; i < _numConfigs; i++) {
          if (_configs[i].type == 0 || _configs[i].type == -1) {
            memcpy(&_configs[i], &newCalc, sizeof(CalculationConfig));
            resetRuntimeState(i);

            mqtt.subscribeToTopic(
              _configs[i].binding.tag,
              _configs[i].binding.isNested
            );

        #if DEBUG_EXPRESSIONS
            Serial.printf("[Manager] Added External '%s' at front slot %d\n", _configs[i].name, i);
        #endif
            return i;
          }
        }
      } else {
        // 2. Logic for Expressions/Continuous: Start from the end and work backwards
        for (int i = _numConfigs - 1; i >= 0; i--) {
          if (_configs[i].type == 0 || _configs[i].type == -1) {
            memcpy(&_configs[i], &newCalc, sizeof(CalculationConfig));
            resetRuntimeState(i);

        #if DEBUG_EXPRESSIONS
            Serial.printf("[Manager] Added Logic '%s' at back slot %d\n", _configs[i].name, i);
        #endif
            return i;
          }
        }
      }

      return -1; // No available slots
    }

    /* Summary of function: Updates an existing calculation and refreshes MQTT subscription if the binding changed */
    void updateCalculation(int index, const CalculationConfig& newCalc, MqttManager& mqtt) {
      if (index >= 0 && index < _numConfigs) {
        memcpy(&_configs[index], &newCalc, sizeof(CalculationConfig));
        resetRuntimeState(index);

        #if DEBUG_EXPRESSIONS
        Serial.printf("[Manager] Slot %d updated and state reset\n", index);
        #endif

        if (_configs[index].type == CALC_EXTERNAL) {
          mqtt.subscribeToTopic(
            _configs[index].binding.tag,
            _configs[index].binding.isNested,
            _configs[index].binding.nestedValueKey
          );
        }
      }
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Returns the result of the last successful calculation.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    float GetLastValue(int index) {
      if (index < 0 || index >= _numConfigs) return NAN;
      return _runtime[index].lastValue;
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Disables a calculation and clears its name and state.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    /**
                                                                                                                                                                                                                                                                                 * Summary of function: Disables a calculation, clears state, and unsubscribes from MQTT if it was an external sensor.
                                                                                                                                                                                                                                                                                 * Updated to handle dynamic cleanup for Grefur-sensor nodes.
                                                                                                                                                                                                                                                                                 */
    void removeCalculation(int index, MqttManager& mqtt) {
      if (index >= 0 && index < _numConfigs) {
        // 1. If this is an external sensor, unsubscribe before wiping the config
        if (_configs[index].type == CALC_EXTERNAL) {
          mqtt.unsubscribeFromTopic(
            _configs[index].binding.tag,
            _configs[index].binding.isNested
          );

          #if DEBUG_EXPRESSIONS
          Serial.printf("[Manager] Unsubscribed from topic: %s\n", _configs[index].binding.tag);
          #endif
        }

        #if DEBUG_EXPRESSIONS
        Serial.printf("[Manager] Deleting calculation '%s' at slot %d\n", _configs[index].name, index);
        #endif

        // 2. Clear the configuration and reset runtime state
        _configs[index].type = 0;
        memset(_configs[index].name, 0, sizeof(_configs[index].name));
        resetRuntimeState(index);
      }
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                             * Finds a calculation by name and returns its last evaluated value.
                                                                                                                                                                                                                                                                                                                                                                                                                                                             * Returns NAN if not found, allowing BindingEngine to try other sources.
                                                                                                                                                                                                                                                                                                                                                                                                                                                             */
    float getCalculationValueByName(const String& name) {
      for (int i = 0; i < _numConfigs; i++) {
        // We only check active slots that have a matching name
        if (_configs[i].type != 0 && String(_configs[i].name) == name) {
          return _runtime[i].lastValue;
        }
      }
      return NAN;
    }

    /* Summary of function: Iterates through configs and registers all external bindings with the MQTT provider */
    void registerExternalSubscriptions(MqttManager& mqtt) {
      for (int i = 0; i < _numConfigs; i++) {
        if (_configs[i].type == CALC_EXTERNAL) {
          // Use the tag as the topic and the nested settings from the binding struct
          mqtt.subscribeToTopic(
            _configs[i].binding.tag,
            _configs[i].binding.isNested,
            _configs[i].binding.nestedValueKey
          );

      #if DEBUG_EXPRESSIONS
          Serial.printf("[Expression Manager] Requesting sub for slot %d: %s\n", i, _configs[i].binding.tag);
      #endif
        }
      }
    }

    /* Summary of function: Pulls data from global MQTT cache and monitors for timeouts for a specific slot.
                                                                                                                                                                                                                                                   Enhanced with diagnostic debugging for the Grefur-SaaS data flow. */
    void handleExternalMessages(int i) {
      uint32_t currentTime = millis();
      uint32_t startMicros = micros();
      auto& cfg = _configs[i];

      String payload;
      uint32_t messageTs;

      // 1. Try to pull latest data from global MqttManager
      if (mqtt.requestExternalValueByTopic(cfg.binding.tag, payload, messageTs)) {
        #if DEBUG_EXPRESSIONS
        Serial.printf("[Manager] Cache Hit | Slot %d | Tag: %s | Cache TS: %u\n", i, cfg.binding.tag, messageTs);
        #endif

        // Only update if we actually have a newer message than last scan
        if (messageTs > cfg.binding.lastUpdateTs) {
          float newValue = payload.toFloat();

          #if DEBUG_EXPRESSIONS
          Serial.printf("[Manager] New Data | Slot %d | %.2f -> %.2f\n", i, cfg.binding.lastValue, newValue);
          #endif

          _runtime[i].lastValue = newValue;
          cfg.binding.lastValue = newValue;
          cfg.binding.lastUpdateTs = messageTs;

          // Update Grefur-SaaS diagnostics
          if (isnan(cfg.binding.minValObserved) || newValue < cfg.binding.minValObserved) {
            cfg.binding.minValObserved = newValue;
          }
          if (isnan(cfg.binding.maxValObserved) || newValue > cfg.binding.maxValObserved) {
            cfg.binding.maxValObserved = newValue;
          }

          cfg.binding.lastScanTimeMs = (micros() - startMicros);
        }
      } else {
        #if DEBUG_EXPRESSIONS
        // Only print this if we have a tag but no data in cache
        if (strlen(cfg.binding.tag) > 0) {
          Serial.printf("[Manager] Cache Miss | Slot %d | No data for topic: %s\n", i, cfg.binding.tag);
        }
        #endif
      }

      // 2. Handle Timeout logic
      uint32_t resetInterval = cfg.binding.isResetIntervalRef
                               ? (uint32_t)getCalculationValueByName(String(cfg.binding.resetIntervalMs))
                               : cfg.binding.resetIntervalMs;

      if (resetInterval > 0 && (currentTime - cfg.binding.lastUpdateTs > resetInterval)) {
        if (_runtime[i].lastValue != cfg.binding.fallbackValue) {
          _runtime[i].lastValue = cfg.binding.fallbackValue;

          #if DEBUG_EXPRESSIONS
          Serial.printf("[Manager] Timeout | Slot %d (%s) | Reset to fallback: %.2f (Age: %u ms)\n",
                        i, cfg.name, cfg.binding.fallbackValue, (currentTime - cfg.binding.lastUpdateTs));
          #endif
        }
      }
    }

  private:
    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Processes event-based expressions. Only updates if change > threshold.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    void evaluateExpression(int index) {
      auto& parentCfg = _configs[index];
      auto& cfg = parentCfg.expression;
      float value = evalExpression(cfg.expression, &_runtime[index]);

      if (!isnan(_runtime[index].lastTriggerValue)) {
        float diff = fabs(value - _runtime[index].lastTriggerValue);
        if (diff < cfg.triggerThreshold) return;
      }

      #if DEBUG_EXPRESSIONS
      Serial.printf("[Calc] %s: Value change -> %.2f\n", parentCfg.name, value);
      #endif

      _runtime[index].lastTriggerValue = value;
      _runtime[index].lastValue = value;
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Processes continuous expressions at fixed intervals. Essential for PID stability.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    void evaluateContinuous(int index, uint32_t deltaMs) {
      auto& parentCfg = _configs[index];
      auto& cfg = parentCfg.continuous;
      static uint32_t elapsed[MAX_CALCULATIONS_ALLOWED] = {0};

      elapsed[index] += deltaMs;
      if (elapsed[index] < cfg.evalIntervalMs) return;

      _runtime[index].lastProcessTime = elapsed[index];
      elapsed[index] = 0;

      float value = evalExpression(cfg.expression, &_runtime[index]);

      if (!isnan(cfg.minValue)) value = fmax(cfg.minValue, value);
      if (!isnan(cfg.maxValue)) value = fmin(cfg.maxValue, value);

      #if DEBUG_EXPRESSIONS
      Serial.printf("[Continuous] %s: Update -> %.2f\n", parentCfg.name, value);
      #endif

      _runtime[index].lastValue = value;
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Recursive parser for handling parentheses and ternary logic.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    float evalExpression(const String& expr, RuntimeState* state) {
      String s = expr;
      s.trim();

      // Handle outermost parentheses to prevent token errors
      if (s.startsWith("(") && s.endsWith(")")) {
        int depth = 0;
        bool splitFound = false;
        for (int i = 0; i < s.length() - 1; i++) {
          if (s[i] == '(') depth++;
          else if (s[i] == ')') depth--;
          if (depth == 0) { splitFound = true; break; }
        }
        if (!splitFound) return evalExpression(s.substring(1, s.length() - 1), state);
      }

      // Handle Ternary Logic (Cond ? True : False)
      int qIndex = findTopLevelQuestion(s);
      if (qIndex != -1) {
        float condValue = evalExpression(s.substring(0, qIndex), state);
        int colonIndex = findMatchingColon(s, qIndex);
        if (colonIndex == -1) return 0.0f;

        return (condValue != 0.0f)
               ? evalExpression(s.substring(qIndex + 1, colonIndex), state)
               : evalExpression(s.substring(colonIndex + 1), state);
      }

      return evalSimpleExpression(s, state);
    }

    int findTopLevelQuestion(const String& s) {
      int depth = 0;
      for (int i = 0; i < s.length(); i++) {
        if (s[i] == '(') depth++;
        else if (s[i] == ')') depth--;
        else if (depth == 0 && s[i] == '?') return i;
      }
      return -1;
    }

    int findMatchingColon(const String& s, int questionIndex) {
      int depth = 0;
      for (int i = questionIndex + 1; i < s.length(); i++) {
        if (s[i] == '(') depth++;
        else if (s[i] == ')') depth--;
        else if (s[i] == '?') depth++;
        else if (depth == 0 && s[i] == ':') return i;
        else if (s[i] == ':') depth--;
      }
      return -1;
    }

    /**
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     * Resolves tokens into actual values using BindingEngine or float conversion.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                     */
    float getValueOrSensor(const String& token) {
      String t = token;
      t.trim();
      if (t.length() == 0) return 0.0f;

      char* endptr;
      float val = strtof(t.c_str(), &endptr);
      if (*endptr != '\0') {
        val = BindingEngine(t);
        #if DEBUG_EXPRESSIONS
        Serial.printf("  [Token Lookup] '%s' -> %.2f\n", t.c_str(), val);
        #endif
      }
      return val;
    }

    /*
                                                                                                                                                                                                                                                                                                                                                                                                                                                                 * Evaluates a single level of an expression.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                 * Handles specialized functions, comparisons, and basic arithmetic.
                                                                                                                                                                                                                                                                                                                                                                                                                                                                 */
    float evalSimpleExpression(const String& expr, RuntimeState* state) {
      String s = expr;
      s.trim();
      if (s.length() == 0) return 0.0f;

      /* Summary of function: Handles unary NOT operator */
      if (s.startsWith("!")) {
        float val = evalExpression(s.substring(1), state);
        return (val == 0.0f) ? 1.0f : 0.0f;
      }

      // 1. Specialized Function Parsing

      if (s.startsWith("DELAY(") && s.endsWith(")")) {
        String inner = s.substring(6, s.length() - 1);

        float args[2] = {0};
        int argCount = 0, lastPos = 0, depth = 0;
        String mode = "";
        String unit = "MS";

        std::vector<String> parts;

        for (int i = 0; i < inner.length(); i++) {
          if (inner[i] == '(') depth++;
          else if (inner[i] == ')') depth--;
          else if (depth == 0 && inner[i] == ',') {
            parts.push_back(inner.substring(lastPos, i));
            lastPos = i + 1;
          }
        }
        parts.push_back(inner.substring(lastPos));

        if (parts.size() >= 2) {
          args[0] = evalExpression(parts[0], state);
          args[1] = evalExpression(parts[1], state);
          argCount = 2;
        }
        if (parts.size() >= 3) {
          String m = parts[2];
          m.trim(); m.replace("\"", ""); m.replace("'", ""); m.toUpperCase();
          mode = m;
        }
        if (parts.size() >= 4) {
          String u = parts[3];
          u.trim(); u.replace("\"", ""); u.replace("'", ""); u.toUpperCase();
          unit = u;
        }

        if (argCount >= 2 && state) {
          float input = args[0];
          uint32_t rawTime = (uint32_t)args[1];

          uint32_t ms = rawTime;
          if      (unit == "S"  || unit == "SEC"    || unit == "SECOND"  || unit == "SECONDS") ms = rawTime * 1000UL;
          else if (unit == "M"  || unit == "MIN"    || unit == "MINUTE"  || unit == "MINUTES") ms = rawTime * 60000UL;
          else if (unit == "H"  || unit == "HOUR"   || unit == "HOURS"                       ) ms = rawTime * 3600000UL;

          uint32_t now = millis();

          if (mode == "ON") {
            if (!state->delayInitialized) {
              state->delayInitialized = true;
              state->delayStartTime = 0;
              state->delayTargetValue = 0.0f;
              return 0.0f;  // alltid start i AV uansett input
            }
            if (input == 0.0f) {
              state->delayStartTime = 0;
              state->delayTargetValue = 0.0f;
              return 0.0f;
            } else {
              if (state->delayStartTime == 0 || state->delayTargetValue != 1.0f) {
                state->delayStartTime = now;
                state->delayTargetValue = 1.0f;
              }
              return (now - state->delayStartTime >= ms) ? 1.0f : 0.0f;
            }

          } else if (mode == "OFF") {
            if (!state->delayInitialized) {
              state->delayInitialized = true;
              state->delayTargetValue = (input != 0.0f) ? 1.0f : 0.0f;
              state->delayStartTime = 0;
              return state->delayTargetValue;
            }
            if (input != 0.0f) {
              state->delayStartTime = 0;
              state->delayTargetValue = 1.0f;
              return 1.0f;
            } else {
              if (state->delayStartTime == 0 || state->delayTargetValue != 0.0f) {
                state->delayStartTime = now;
                state->delayTargetValue = 0.0f;
              }
              return (now - state->delayStartTime >= ms) ? 0.0f : 1.0f;
            }

          } else {
            if (!state->delayInitialized) {
              state->delayInitialized = true;
              state->delayTargetValue = input;
              state->delayStartTime = now;
              return input;  // sett utgangen til faktisk input ved oppstart, ingen forsinkelse
            }
            if (fabsf(input - state->delayTargetValue) > 0.01f) {
              state->delayTargetValue = input;
              state->delayStartTime = now;
            }
            if (now - state->delayStartTime >= ms) {
              return input;
            }
            return isnan(state->lastValue) ? input : state->lastValue;
          }
        }
      }




      /* Summary of function: CYCLE(on, off) - Toggles between 1.0 and 0.0 based on asymmetrical timing */
      if (s.startsWith("CYCLE(") && s.endsWith(")")) {
        String inner = s.substring(6, s.length() - 1);
        int comma = inner.indexOf(',');

        uint32_t onTime, offTime;
        if (comma != -1) {
          onTime = (uint32_t)evalExpression(inner.substring(0, comma), state);
          offTime = (uint32_t)evalExpression(inner.substring(comma + 1), state);
        } else {
          onTime = offTime = (uint32_t)evalExpression(inner, state);
        }

        uint32_t totalPeriod = onTime + offTime;
        if (totalPeriod == 0) return 0.0f;

        // Calculate position within the current cycle period
        uint32_t currentCyclePos = millis() % totalPeriod;

        // If we are within the first part of the period, return 1.0 (On)
        return (currentCyclePos < onTime) ? 1.0f : 0.0f;
      }

      /* Summary of function: HYST(input, highThreshold, lowThreshold) - Schmit-trigger logic to prevent flapping */
      if (s.startsWith("HYST(") && s.endsWith(")")) {
        String inner = s.substring(5, s.length() - 1);
        float args[3] = {0};
        int argCount = 0, lastPos = 0, depth = 0;

        for (int i = 0; i < inner.length() && argCount < 3; i++) {
          if (inner[i] == '(') depth++;
          else if (inner[i] == ')') depth--;
          else if (depth == 0 && inner[i] == ',') {
            args[argCount++] = evalExpression(inner.substring(lastPos, i), state);
            lastPos = i + 1;
          }
        }
        if (argCount < 3) args[argCount++] = evalExpression(inner.substring(lastPos), state);

        if (argCount == 3 && state) {
          float val = args[0];            // Nåværende verdi (f.eks. Temp)
          float highThreshold = args[1];  // Aktiver ved (f.eks. 25.0)
          float lowThreshold = args[2];   // Deaktiver ved (f.eks. 22.0)

          bool lastState = (state->lastValue > 0.5f);

          if (lastState) {
            // Hvis den er PÅ, forblir den PÅ til den synker under lowThreshold
            return (val > lowThreshold) ? 1.0f : 0.0f;
          } else {
            // Hvis den er AV, forblir den AV til den stiger over highThreshold
            return (val > highThreshold) ? 1.0f : 0.0f;
          }
        }
        return 0.0f;
      }

      // RAMP(target, rateUp, rateDown) or RAMP(target, rate)
      if (s.startsWith("RAMP(") && s.endsWith(")")) {

        String inner = s.substring(5, s.length() - 1);

        float args[3] = {0};
        int argCount = 0, lastPos = 0, depth = 0;

        for (int i = 0; i < inner.length() && argCount < 3; i++) {
          if (inner[i] == '(') depth++;
          else if (inner[i] == ')') depth--;
          else if (depth == 0 && inner[i] == ',') {
            args[argCount++] = evalExpression(inner.substring(lastPos, i), state);
            lastPos = i + 1;
          }
        }

        args[argCount++] = evalExpression(inner.substring(lastPos), state);

        if (state && (argCount == 2 || argCount == 3)) {

          float target = args[0];

          float rateUp, rateDown;

          if (argCount == 2) {
            rateUp = fabsf(args[1]);
            rateDown = fabsf(args[1]);
          } else {
            rateUp = fabsf(args[1]);
            rateDown = fabsf(args[2]);
          }

          uint32_t now = millis();

          // first run init
          if (isnan(state->lastValue)) {
            state->lastValue = target;
            state->lastProcessTime = now;
            return target;
          }

          float dt = (now - state->lastProcessTime) / 1000.0f;
          state->lastProcessTime = now;

          float current = state->lastValue;
          float delta = target - current;

          float maxStep;

          if (delta > 0)
            maxStep = rateUp * dt;
          else
            maxStep = rateDown * dt;

          if (fabsf(delta) <= maxStep)
            current = target;
          else
            current += (delta > 0) ? maxStep : -maxStep;

          state->lastValue = current;

          return current;
        }

        return 0.0f;
      }

      // AVG(n1, n2, n3...): Arithmetic average of all arguments
      if (s.startsWith("AVG(") && s.endsWith(")")) {
        String inner = s.substring(4, s.length() - 1);
        float sum = 0;
        int count = 0, lastPos = 0, depth = 0;

        for (int i = 0; i < inner.length(); i++) {
          if (inner[i] == '(') depth++;
          else if (inner[i] == ')') depth--;
          else if (depth == 0 && inner[i] == ',') {
            sum += evalExpression(inner.substring(lastPos, i), state);
            count++;
            lastPos = i + 1;
          }
        }
        sum += evalExpression(inner.substring(lastPos), state);
        count++;
        return (count > 0) ? (sum / count) : 0.0f;
      }

      // ROUND(value, decimals)
      if (s.startsWith("ROUND(") && s.endsWith(")")) {
        String inner = s.substring(6, s.length() - 1);
        int comma = inner.lastIndexOf(',');
        if (comma != -1) {
          float val = evalExpression(inner.substring(0, comma), state);
          int dec = (int)evalExpression(inner.substring(comma + 1), state);
          float factor = pow(10, dec);
          return roundf(val * factor) / factor;
        }
      }

      // PID Controller: PID(input, setpoint, Kp, Ki, Kd)
      if (s.startsWith("PID(") && s.endsWith(")")) {
        String inner = s.substring(4, s.length() - 1);
        float args[5] = {0};
        int argCount = 0, lastPos = 0, depth = 0;

        for (int i = 0; i < inner.length() && argCount < 5; i++) {
          if (inner[i] == '(') depth++;
          else if (inner[i] == ')') depth--;
          else if (depth == 0 && inner[i] == ',') {
            args[argCount++] = evalExpression(inner.substring(lastPos, i), state);
            lastPos = i + 1;
          }
        }
        if (argCount < 5) args[argCount++] = evalExpression(inner.substring(lastPos), state);

        if (state && argCount == 5) {
          float error = args[1] - args[0]; // Setpoint - Input
          float dt = (state->lastProcessTime > 0) ? state->lastProcessTime / 1000.0f : 1.0f;

          state->integralSum += error * dt;
          float dTerm = (error - state->lastError) / dt;
          state->lastError = error;

          float result = (args[2] * error) + (args[3] * state->integralSum) + (args[4] * dTerm);
          return result;
        }
        return 0.0f;
      }

      // LIMIT Function: LIMIT(value, min, max)
      if (s.startsWith("LIMIT(") && s.endsWith(")")) {
        String inner = s.substring(6, s.length() - 1);
        int c1 = -1, c2 = -1, depth = 0;
        for (int i = 0; i < inner.length(); i++) {
          if (inner[i] == '(') depth++;
          else if (inner[i] == ')') depth--;
          else if (depth == 0 && inner[i] == ',') {
            if (c1 == -1) c1 = i; else c2 = i;
          }
        }
        if (c1 != -1 && c2 != -1) {
          float v = evalExpression(inner.substring(0, c1), state);
          float lo = evalExpression(inner.substring(c1 + 1, c2), state);
          float hi = evalExpression(inner.substring(c2 + 1), state);
          return (v < lo) ? lo : (v > hi) ? hi : v;
        }
      }

      // CURVE Function: Linear interpolation between X,Y pairs
      if (s.startsWith("CURVE(") && s.endsWith(")")) {
        String inner = s.substring(6, s.length() - 1);
        const int maxCurveArgs = 32;
        float args[maxCurveArgs];
        int argCount = 0, lastPos = 0, depth = 0;

        for (int i = 0; i < inner.length() && argCount < maxCurveArgs; i++) {
          if (inner[i] == '(') depth++;
          else if (inner[i] == ')') depth--;
          else if (depth == 0 && inner[i] == ',') {
            args[argCount++] = evalExpression(inner.substring(lastPos, i), state);
            lastPos = i + 1;
          }
        }
        if (argCount < maxCurveArgs) args[argCount++] = evalExpression(inner.substring(lastPos), state);

        if (argCount >= 6 && (argCount % 2 == 0)) {
          float v = args[0];
          bool applyLimit = (args[argCount - 1] != 0.0f);
          int numPoints = (argCount - 2) / 2;

          if (v <= args[1]) {
            if (applyLimit) return args[2];
            float slope = (args[4] - args[2]) / (args[3] - args[1]);
            return args[2] + (v - args[1]) * slope;
          }

          for (int i = 0; i < numPoints - 1; i++) {
            int xIdx = 1 + (i * 2);
            int yIdx = 2 + (i * 2);
            int nextXIdx = xIdx + 2;
            int nextYIdx = yIdx + 2;
            if (v <= args[nextXIdx]) {
              float slope = (args[nextYIdx] - args[yIdx]) / (args[nextXIdx] - args[xIdx]);
              return args[yIdx] + (v - args[xIdx]) * slope;
            }
          }

          int lastXIdx = 1 + ((numPoints - 1) * 2);
          int lastYIdx = 2 + ((numPoints - 1) * 2);
          if (applyLimit) return args[lastYIdx];
          int prevXIdx = lastXIdx - 2;
          int prevYIdx = lastYIdx - 2;
          float lastSlope = (args[lastYIdx] - args[prevYIdx]) / (args[lastXIdx] - args[prevXIdx]);
          return args[lastYIdx] + (v - args[lastXIdx]) * lastSlope;
        }
        return 0.0f;
      }

      // Absolute Value: ABS(value)
      if (s.startsWith("ABS(") && s.endsWith(")")) {
        return fabsf(evalExpression(s.substring(4, s.length() - 1), state));
      }

      // Square Root: SQRT(value)
      if (s.startsWith("SQRT(") && s.endsWith(")")) {
        float val = evalExpression(s.substring(5, s.length() - 1), state);
        return (val < 0) ? 0.0f : sqrtf(val);
      }

      // 2. Logical Operators
      int depth = 0;
      for (int i = 0; i < s.length(); i++) {
        if (s[i] == '(') depth++;
        else if (s[i] == ')') depth--;
        else if (depth == 0) {
          if (s.substring(i, i + 2) == "&&") {
            bool left = (evalExpression(s.substring(0, i), state) != 0.0f);
            if (!left) return 0.0f; // Short-circuit
            return (evalExpression(s.substring(i + 2), state) != 0.0f) ? 1.0f : 0.0f;
          }
          if (s.substring(i, i + 2) == "||") {
            bool left = (evalExpression(s.substring(0, i), state) != 0.0f);
            if (left) return 1.0f; // Short-circuit
            return (evalExpression(s.substring(i + 2), state) != 0.0f) ? 1.0f : 0.0f;
          }
        }
      }

      // 2. Comparison Operators (Evaluated after functions)
      depth = 0;
      for (int i = 0; i < s.length(); i++) {
        if (s[i] == '(') depth++;
        else if (s[i] == ')') depth--;
        else if (depth == 0) {
          String op2 = s.substring(i, i + 2);
          if (op2 == "==") return evalExpression(s.substring(0, i), state) == evalExpression(s.substring(i + 2), state);
          if (op2 == "!=") return evalExpression(s.substring(0, i), state) != evalExpression(s.substring(i + 2), state);
          if (op2 == ">=") return evalExpression(s.substring(0, i), state) >= evalExpression(s.substring(i + 2), state);
          if (op2 == "<=") return evalExpression(s.substring(0, i), state) <= evalExpression(s.substring(i + 2), state);
          if (s[i] == '>') return evalExpression(s.substring(0, i), state) > evalExpression(s.substring(i + 1), state);
          if (s[i] == '<') return evalExpression(s.substring(0, i), state) < evalExpression(s.substring(i + 1), state);
        }
      }

      // 3. Mathematical Operators (Recursion for arithmetic)
      // Order of operations: + and - are checked first to be evaluated last
      int plusIdx = -1, minusIdx = -1, multIdx = -1, divIdx = -1;
      depth = 0;
      for (int i = s.length() - 1; i >= 0; i--) {
        if (s[i] == ')') depth++;
        else if (s[i] == '(') depth--;
        else if (depth == 0) {
          if (plusIdx == -1 && s[i] == '+') plusIdx = i;
          // Ensure '-' is an operator and not a negative sign (must follow a value/sensor/parenthesis)
          if (minusIdx == -1 && s[i] == '-' && i > 0 && (isAlphaNumeric(s[i-1]) || s[i-1] == '_' || s[i-1] == ')')) minusIdx = i;
          if (multIdx == -1 && s[i] == '*') multIdx = i;
          if (divIdx == -1 && s[i] == '/') divIdx = i;
        }
      }

      if (plusIdx != -1) return evalExpression(s.substring(0, plusIdx), state) + evalExpression(s.substring(plusIdx + 1), state);
      if (minusIdx != -1) return evalExpression(s.substring(0, minusIdx), state) - evalExpression(s.substring(minusIdx + 1), state);
      if (multIdx != -1) return evalExpression(s.substring(0, multIdx), state) * evalExpression(s.substring(multIdx + 1), state);
      if (divIdx != -1) {
        float den = evalExpression(s.substring(divIdx + 1), state);
        return (den == 0) ? 0.0f : evalExpression(s.substring(0, divIdx), state) / den;
      }

      // 4. Final Fallback: Numeric Value or Sensor Lookup
      return getValueOrSensor(s);
    }



    /**
                                                                                                                                                                                                                                                                                                                         * Summary of function: Updates the value of an external binding slot and refreshes its timestamp.
                                                                                                                                                                                                                                                                                                                         * Used by the communication layer when a bound MQTT message or internal tag update arrives.
                                                                                                                                                                                                                                                                                                                         */
    void updateExternalValue(int index, float newValue) {
      if (index >= 0 && index < _numConfigs && _configs[index].type == CALC_EXTERNAL) {
        uint32_t startMicros = micros();

        _configs[index].binding.lastValue = newValue;
        _configs[index].binding.lastUpdateTs = millis();
        _runtime[index].lastValue = newValue;

        // Track statistics for Grefur-SaaS diagnostics
        if (isnan(_configs[index].binding.minValObserved) || newValue < _configs[index].binding.minValObserved)
          _configs[index].binding.minValObserved = newValue;
        if (isnan(_configs[index].binding.maxValObserved) || newValue > _configs[index].binding.maxValObserved)
          _configs[index].binding.maxValObserved = newValue;

        _configs[index].binding.lastScanTimeMs = (micros() - startMicros); // Track overhead
      }
    }
};

ExpressionManager expressionManager(config.calculations, MAX_CALCULATIONS_ALLOWED);




/* Summary of class: Manages hardware actuators by mirroring values from the ExpressionManager and tracking units for display. */
class ActuatorManager {
  private:
    EEPROMLayout& _config;
    bool _changed;

    // Track initialization in RAM only to avoid pinMode flicker
    bool _initializedPins[MAX_ACTUATORS_ALLOWED];

  public:
    ActuatorManager(EEPROMLayout& config) : _config(config), _changed(false) {
      // Ensure all slots are marked as uninitialized on boot
      for(int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) _initializedPins[i] = false;
    }

    /* Summary of function: Adds a new actuator to the first available EEPROM slot */
    int addActuator(const ActuatorConfig& newActuator) {
      for (int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) {
        if (_config.actuators[i].type == ACTUATOR_UNCONFIGURED) {

          memcpy(&_config.actuators[i], &newActuator, sizeof(ActuatorConfig));

          // Ensure runtime variables are initialized
          _config.actuators[i].runtimeValue = -1.0f;
          _config.actuators[i].lastUpdateTs = 0;
          _config.actuators[i].lastPublishedValue = -1.0f;
          _config.actuators[i].lastPublishTs = 0;

          _initializedPins[i] = false; // Reset init state for this slot

          _changed = true;

          #if DEBUG_ACTUATOR_MANAGER
          Serial.printf("[Actuator] Added '%s' at slot %d mirroring source '%s'\n",
                        _config.actuators[i].name, i, _config.actuators[i].sourceName);
          #endif
          return i;
        }
      }
      return -1;
    }

    /* Summary of function: Removes actuator and marks the slot as unconfigured */
    bool removeActuator(int id) {
      if (id < 0 || id >= MAX_ACTUATORS_ALLOWED) return false;

      memset(&_config.actuators[id], 0, sizeof(ActuatorConfig));
      _config.actuators[id].type = ACTUATOR_UNCONFIGURED;

      _changed = true;

      #if DEBUG_ACTUATOR_MANAGER
      Serial.printf("[Actuator] Removed slot %d\n", id);
      #endif
      return true;
    }

    /* Summary of function: Drives pins according to publishPolicy, calling pinMode only once per session */
    void updateActuators(ExpressionManager& expManager) {
      uint32_t currentTime = millis();
      for (int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) {
        ActuatorConfig& act = _config.actuators[i];
        if (act.type == ACTUATOR_UNCONFIGURED) continue;

        float targetValue = expManager.getCalculationValueByName(String(act.sourceName));
        if (isnan(targetValue)) continue;

        // Evaluate publish policy
        /*float delta = abs(targetValue - act.runtimeValue);
                                                                                                        uint32_t elapsed = currentTime - act.lastUpdateTs;
                                                                                                        bool shouldUpdate = false;

                                                                                                        switch (act.publishPolicy.trigger) {
                                                                                                          case PUBLISH_ON_THRESHOLD_OR_INTERVAL:
                                                                                                            shouldUpdate = (delta >= act.publishPolicy.changeThreshold)
                                                                                                                           || (elapsed >= act.publishPolicy.intervalMs);
                                                                                                            break;
                                                                                                          case PUBLISH_ON_CHANGE_OF_THRESHOLD:
                                                                                                            shouldUpdate = (delta >= act.publishPolicy.changeThreshold);
                                                                                                            break;
                                                                                                          case PUBLISH_ON_INTERVAL:
                                                                                                            shouldUpdate = (elapsed >= act.publishPolicy.intervalMs);
                                                                                                            break;
                                                                                                          default:
                                                                                                            shouldUpdate = true;
                                                                                                            break;
                                                                                                        }

                                                                                                        if (!shouldUpdate) continue;*/

        // Drive hardware
        if (act.type == ACTUATOR_DIGITAL) {
          bool state = (targetValue > 0.5f);

          //Invert
          if (act.digital.inverted) state = !state;

          if (!_initializedPins[i]) {
            pinMode(act.digital.digitalPin, OUTPUT);
            _initializedPins[i] = true;
          }

          analogWrite(act.digital.digitalPin, state ?  0: 255);

        }
        else if (act.type == ACTUATOR_ANALOG) {
          float clampedValue = constrain(targetValue, act.analog.minValue, act.analog.maxValue);
          float targetVoltage = act.analog.voltageMin + (clampedValue / 100.0f) * (act.analog.voltageMax - act.analog.voltageMin);
          int pwmValue = (int)((targetVoltage / 3.3f) * 255.0f);
          pwmValue = constrain(pwmValue, 0, 255);
          if (act.digital.inverted) pwmValue = 255 - pwmValue;
          if (!_initializedPins[i]) {
            pinMode(act.analog.analogPin, OUTPUT);
            _initializedPins[i] = true;
          }
          analogWrite(act.analog.analogPin, pwmValue);
        }

        act.runtimeValue = targetValue;
        act.lastUpdateTs = currentTime;
      }
    }

    bool saveChanges() {
      if (!_changed) return true;
      _changed = false;
      return saveConfig();
    }

    bool shouldPublishActuator(int i, uint32_t currentTime) {
      if (getType(i) == ACTUATOR_UNCONFIGURED) return false;

      float actVal = getValue(i);
      if (isnan(actVal) || actVal < 0) return false;

      const char* actName = _config.actuators[i].name;
      if (strlen(actName) == 0) return false;

      ActuatorConfig& act = _config.actuators[i];
      float delta = abs(actVal - act.lastPublishedValue);
      uint32_t elapsed = currentTime - act.lastPublishTs;

      switch (act.publishPolicy.trigger) {
        case PUBLISH_ON_THRESHOLD_OR_INTERVAL:
          return (delta >= act.publishPolicy.changeThreshold) || (elapsed >= act.publishPolicy.intervalMs);
        case PUBLISH_ON_CHANGE_OF_THRESHOLD:
          return (delta >= act.publishPolicy.changeThreshold);
        case PUBLISH_ON_INTERVAL:
          return (elapsed >= act.publishPolicy.intervalMs);
        default:
          return true;
      }
    }

    void setLastPublished(int i, float val, uint32_t ts) {
      if (i < 0 || i >= MAX_ACTUATORS_ALLOWED) return;
      _config.actuators[i].lastPublishedValue = val;
      _config.actuators[i].lastPublishTs = ts;
    }

    /* Summary of function: Returns the current runtime value of a specific actuator */
    float getValue(int id) const {
      if (id < 0 || id >= MAX_ACTUATORS_ALLOWED) return NAN;
      const ActuatorConfig& act = _config.actuators[id];
      if (act.type == ACTUATOR_DIGITAL && act.digital.inverted) {
        return (act.runtimeValue > 0.5f) ? 0.0f : 1.0f;
      }
      return act.runtimeValue;
    }

    /* Summary of function: Returns the numeric type of the actuator (e.g., 20 for Digital, 21 for Analog). */
    int getType(int id) const {
      if (id < 0 || id >= MAX_ACTUATORS_ALLOWED) return 0;
      return _config.actuators[id].type;
    }
    /* Summary of function: Returns the unit associated with the actuator type */
    String getUnit(int id) const {
      if (id < 0 || id >= MAX_ACTUATORS_ALLOWED) return "";
      ActuatorConfig& act = _config.actuators[id];

      if (act.type == ACTUATOR_DIGITAL) return "BOOL"; // Or "ON/OFF"
      if (act.type == ACTUATOR_ANALOG) return String(act.analog.unit);

      return "";
    }

    /* Summary of function: Returns JSON representation of all active actuators for the dashboard */
    String retrieveAllActuators() {
      if (!OUTPUT_MODULE) return String();
      DynamicJsonDocument doc(2048);
      JsonArray actuators = doc.createNestedArray("actuators");

      for (int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) {
        ActuatorConfig& act = _config.actuators[i];
        if (act.type != ACTUATOR_UNCONFIGURED) {
          JsonObject obj = actuators.createNestedObject();
          obj["id"] = i;
          obj["type"] = act.type;
          obj["name"] = act.name;
          obj["source"] = act.sourceName;
          obj["val"] = act.runtimeValue;
          obj["unit"] = getUnit(i); // Included unit in JSON output

          switch (act.type) {
            case ACTUATOR_DIGITAL:
              obj["details"] = String("Pin ") + act.digital.digitalPin +
                               (act.digital.inverted ? " (inverted)" : "");
              break;
            case ACTUATOR_ANALOG:
              obj["details"] = String("Pin ") + act.analog.analogPin + " (" +
                               act.analog.minValue + "-" + act.analog.maxValue + " " + act.analog.unit + ")";
              break;
          }
        }
      }
      String output;
      serializeJson(doc, output);
      return output;
    }

    /* Summary of function: Generates metadata for Web UI forms. */
    String retrieveActuatorTypeFieldsAsJson() {
      DynamicJsonDocument doc(1024);
      JsonObject fields = doc.createNestedObject("fields");
      JsonObject labels = doc.createNestedObject("labels");
      JsonObject placeholders = doc.createNestedObject("placeholders");

      fields["sourceName"] = "text";
      labels["sourceName"] = "Mirror Source Name";
      placeholders["sourceName"] = "e.g., TargetTemp_Calc";

      JsonObject digital = fields.createNestedObject("digital");
      digital["digitalPin"] = "number";
      digital["inverted"] = "boolean";
      labels["digitalPin"] = "GPIO Pin";
      labels["inverted"] = "Active Low";

      JsonObject analog = fields.createNestedObject("analog");
      analog["analogPin"] = "number";
      analog["minValue"] = "number";
      analog["maxValue"] = "number";
      analog["unit"] = "text";

      labels["analogPin"] = "PWM Pin";
      labels["minValue"] = "Min Logic Value";
      labels["maxValue"] = "Max Logic Value";
      labels["unit"] = "Unit";

      String output;
      serializeJson(doc, output);
      return output;
    }

    /* Summary of function: Returns count of configured actuators */
    int getConfiguredActuatorCount() const {
      int count = 0;
      for (int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) {
        if (_config.actuators[i].type != ACTUATOR_UNCONFIGURED) count++;
      }
      return count;
    }

    /* Summary of function: Resets all actuator slots to unconfigured state in EEPROM memory */
    void initializeActuatorSlots() {
      for (int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) {
        _config.actuators[i].type = ACTUATOR_UNCONFIGURED;
        memset(_config.actuators[i].name, 0, sizeof(_config.actuators[i].name));
        memset(_config.actuators[i].sourceName, 0, sizeof(_config.actuators[i].sourceName));
        _config.actuators[i].runtimeValue = 0.0f;
        _config.actuators[i].lastUpdateTs = 0;
        memset(&_config.actuators[i].digital, 0, sizeof(_config.actuators[i].digital));
        memset(_config.actuators[i]._reserved, 0, sizeof(_config.actuators[i]._reserved));
      }
      _changed = true;

      #if DEBUG_ACTUATOR_MANAGER
      Serial.println("[Actuator] All slots initialized to UNCONFIGURED");
      #endif
    }
};

ActuatorManager actuatorManager(config);






/**
 * The BindingEngine resolves tokens to numeric values.
 * Handles local sensors, expressions, and remote Grefur nodes using the {nodeId}/{value} syntax.
 */
float BindingEngine(const String& name) {
  if (name.length() == 0) return NAN;

  // 1. Check for Remote Binding Syntax (nodeId/valueName)
  int slashIdx = name.indexOf('/');
  if (slashIdx != -1) {
    String nodeId = name.substring(0, slashIdx);
    String valueName = name.substring(slashIdx + 1);

    // If nodeId matches our own name, treat it as a local lookup
    if (nodeId == String(generateAPName())) {
      return BindingEngine(valueName);
    }

    // Otherwise, fetch from the RemoteNodeManager cache
    // This cache is populated by MQTT subscriptions
    #if DEBUG_BINDING_ENGINE
    Serial.printf("[BINDING] Requesting remote value '%s' from node '%s'\n", valueName.c_str(), nodeId.c_str());
    #endif

    // return remoteNodeManager.getValue(nodeId, valueName);
    return NAN; // Placeholder until RemoteNodeManager is active
  }

  // 2. SOURCE: Local Physical Sensors
  float val = sensorManager.retrieveLastValueByName(name);
  if (!isnan(val)) return val;

  // 3. SOURCE: Local Calculated Expressions
  val = expressionManager.getCalculationValueByName(name);
  if (!isnan(val)) return val;

  // 4. SOURCE: Numeric Constants
  char* endPtr;
  float numeric = strtof(name.c_str(), &endPtr);
  if (endPtr != name.c_str()) {
    return numeric;
  }

  return NAN;
}

/* Summary of function: Validates if a binding tag is locally available or follows the required remote format (node/sensor) */
bool validateBindingTag(const String& tag, String& outError) {
  if (tag.length() == 0) {
    outError = "Binding tag cannot be empty.";
    return false;
  }

  // 1. Check if it exists as a Local Physical Sensor
  float val = sensorManager.retrieveLastValueByName(tag);
  if (!isnan(val)) return true;

  // 2. Check if it exists as a Local Calculation
  val = expressionManager.getCalculationValueByName(tag);
  if (!isnan(val)) return true;

  // 3. Check if it is a Numeric Constant (some users might bind to a fixed value)
  char* endPtr;
  strtof(tag.c_str(), &endPtr);
  if (endPtr != tag.c_str()) return true;

  // 4. Check for Remote Binding Syntax (must contain at least one '/')
  // Example: "kitchen-node/temperature"
  if (tag.indexOf('/') != -1) {
    return true;
  }

  // If none of the above, it's an orphaned or invalid reference
  outError = "Reference '" + tag + "' not found locally and lacks '/' for remote binding.";
  return false;
}






/**
 * BACKUP MANAGER
 *
 *
 *
 */


class BackupManager {
  private:
    static EEPROMLayout config;

    // Simple Base64 decoding table
    static const char* base64_chars;
    static bool isBase64(char c) {
      return (isalnum(c) || (c == '+') || (c == '/'));
    }

    static int base64Decode(const char* input, uint8_t* output, size_t outLen) {
      int inLen = strlen(input);
      int i = 0, j = 0;
      int in_ = 0;
      uint8_t char_array_4[4], char_array_3[3];

      int outPos = 0;
      while (inLen-- && (input[in_] != '=') && isBase64(input[in_])) {
        char_array_4[i++] = input[in_]; in_++;
        if (i == 4) {
          for (i = 0; i < 4; i++)
            char_array_4[i] = strchr(base64_chars, char_array_4[i]) - base64_chars;

          char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
          char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
          char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

          for (i = 0; i < 3; i++) {
            if (outPos < outLen) output[outPos++] = char_array_3[i];
          }
          i = 0;
        }
      }

      if (i) {
        for (j = i; j < 4; j++)
          char_array_4[j] = 0;

        for (j = 0; j < 4; j++)
          char_array_4[j] = strchr(base64_chars, char_array_4[j]) - base64_chars;

        char_array_3[0] = ( char_array_4[0] << 2 ) + ( (char_array_4[1] & 0x30) >> 4 );
        char_array_3[1] = ( (char_array_4[1] & 0xf) << 4 ) + ( (char_array_4[2] & 0x3c) >> 2 );
        char_array_3[2] = ( (char_array_4[2] & 0x3) << 6 ) + char_array_4[3];

        for (j = 0; (j < i - 1) && (outPos < outLen); j++) output[outPos++] = char_array_3[j];
      }

      return outPos; // return decoded length
    }

  public:
    static void begin() {
      EEPROM.begin(sizeof(EEPROMLayout));
      EEPROM.get(0, config);

      if (config.header.magic != 0xAA55AA55) {
        Serial.println("Invalid EEPROM config, restarting ESP...");
        delay(2000);
        ESP.restart();
      } else {
        Serial.println("Config loaded from EEPROM successfully");
      }
    }

    static String generateBackupJsonBase64() {
      const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&config);
      size_t len = sizeof(EEPROMLayout);

      String b64 = base64::encode(ptr, len); // fortsatt bruk base64::encode for encoding
      DynamicJsonDocument doc(1024);
      doc["data"] = b64;

      String output;
      serializeJson(doc, output);
      return output;
    }

    static bool restoreFromBackupJsonBase64(const String& jsonString) {
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, jsonString);
      if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
        return false;
      }

      const char* b64 = doc["data"];
      if (!b64) {
        Serial.println("Missing Base64 data");
        return false;
      }

      uint8_t buffer[sizeof(EEPROMLayout)];
      int decodedLen = base64Decode(b64, buffer, sizeof(buffer));
      if (decodedLen != sizeof(EEPROMLayout)) {
        Serial.println("Base64 decode size mismatch");
        return false;
      }

      memcpy(&config, buffer, sizeof(EEPROMLayout));

      if (config.header.magic != 0xAA55AA55) {
        Serial.println("Invalid magic number after restore");
        return false;
      }

      EEPROM.put(0, config);
      return EEPROM.commit();
    }

    static const EEPROMLayout& getConfig() {
      return config;
    }
};


// Static member definition
EEPROMLayout BackupManager::config;
const char* BackupManager::base64_chars =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
  "abcdefghijklmnopqrstuvwxyz"
  "0123456789+/";



/**
   @brief CertificateManager handles storing, validating, and generating
          certificates on ESP8266/ESP32 using SPIFFS.

   Features:
    - Add or update device certificate, private key, and optional root CA
    - Remove existing certificates
    - Validate stored certificate format
    - Generate self-signed RSA 2048 certificate
    - Retrieve a summary of installed certificates
*/

/*
class CertificateManager {
  private:
    const char* certPath = "/cert.pem";
    const char* keyPath  = "/key.pem";
    const char* caPath   = "/rootCA.pem";

  public:
    CertificateManager() {}

    // Must be called in setup()
    bool begin() {
      if (!SPIFFS.begin()) {
        Serial.println("Failed to mount SPIFFS");
        return false;
      }
      return true;
    }

    /**
       @brief Add or update certificate, key, and optional root CA
       @param cert PEM-encoded certificate string
       @param key PEM-encoded private key string
       @param rootCA Optional PEM-encoded root CA
       @return Status message**//*

    String addCertificate(const char* cert, const char* key, const char* rootCA = nullptr) {
      if (!saveFile(certPath, cert)) return "Failed to save cert";
      if (!saveFile(keyPath, key)) return "Failed to save key";
      if (rootCA && !saveFile(caPath, rootCA)) return "Failed to save rootCA";
      return "Certificate, key, and root CA stored successfully.";
    }

    // Remove certs
    String removeCertificate() {
      if (SPIFFS.exists(certPath)) SPIFFS.remove(certPath);
      if (SPIFFS.exists(keyPath)) SPIFFS.remove(keyPath);
      if (SPIFFS.exists(caPath)) SPIFFS.remove(caPath);
      return "Certificate removed successfully.";
    }

    // Validate stored certificate
    bool validateCertificate() {
      if (!SPIFFS.exists(certPath) || !SPIFFS.exists(keyPath)) return false;
      File f = SPIFFS.open(certPath, "r");
      if (!f) return false;
      String content = f.readString();
      f.close();
      return content.indexOf("BEGIN CERTIFICATE") >= 0;
    }

    // Get certificate summary
    String getCertificateSummary() {
      String summary;
      summary += "Cert present: " + String(SPIFFS.exists(certPath) ? "Yes" : "No") + "\n";
      summary += "Key present: " + String(SPIFFS.exists(keyPath) ? "Yes" : "No") + "\n";
      summary += "RootCA present: " + String(SPIFFS.exists(caPath) ? "Yes" : "No") + "\n";
      summary += "Valid PEM format: " + String(validateCertificate() ? "Yes" : "No") + "\n";
      return summary;
    }

    /**
       @brief Generate a self-signed RSA 2048 certificate and store in SPIFFS
       @param commonName CN field for certificate
       @param validDays Validity duration in days
       @return true if generation and storage succeeded
    */
/*bool generateSelfSignedCert(String commonName = "ESPDevice", int validDays = 365) {
      // Create key pair
      BearSSL::PrivateKey key;
      key.init();
      key.generate(2048); // 2048-bit RSA

      // Build self-signed certificate
      BearSSL::X509List certList;
      BearSSLHelpers::createSelfSignedCert(certList, key, commonName, validDays);

      String pemCert = certList.toPEM();
      String pemKey = key.toPEM();

      // Store in SPIFFS
      String result = addCertificate(pemCert.c_str(), pemKey.c_str());
      Serial.println(result);
      return validateCertificate();
      }*/
/*
  private:
    // Save string to SPIFFS file
    // Helper: Save string content to SPIFFS file
    bool saveFile(const char* path, const char* content) {
      File f = SPIFFS.open(path, "w");
      if (!f) return false;
      f.print(content);
      f.close();
      return true;
    }
};

// Global instance
CertificateManager certificateManager;
*/
















/*
  // Views / HTML / graphical



*/

// Returns string of the logo as SVG
String getLogoSvg() {
  return String(
           "<svg width=\"50\" height=\"50\" viewBox=\"0 0 106 120\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\">"
           "<path d=\"M79.2727 39.25C78.4773 36.4848 77.3599 34.0417 75.9205 31.9205C74.4811 29.7614 72.7197 27.9432 70.6364 26.4659C68.5909 24.9508 66.2424 23.7955 63.5909 23C60.9773 22.2045 58.0795 21.8068 54.8977 21.8068C48.9508 21.8068 43.7235 23.2841 39.2159 26.2386C34.7462 29.1932 31.2614 33.4924 28.7614 39.1364C26.2614 44.7424 25.0114 51.5985 25.0114 59.7045C25.0114 67.8106 26.2424 74.7045 28.7045 80.3864C31.1667 86.0682 34.6515 90.4053 39.1591 93.3977C43.6667 96.3523 48.9886 97.8295 55.125 97.8295C60.6932 97.8295 65.447 96.8447 69.3864 94.875C73.3636 92.8674 76.3939 90.0455 78.4773 86.4091C80.5985 82.7727 81.6591 78.4735 81.6591 73.5114L86.6591 74.25H56.6591V55.7273H105.352V70.3864C105.352 80.6136 103.193 89.4015 98.875 96.75C94.5568 104.061 88.6098 109.705 81.0341 113.682C73.4583 117.621 64.7841 119.591 55.0114 119.591C44.1023 119.591 34.5189 117.186 26.2614 112.375C18.0038 107.527 11.5644 100.652 6.94318 91.75C2.35985 82.8106 0.0681819 72.2045 0.0681819 59.9318C0.0681819 50.5 1.43182 42.0909 4.15909 34.7045C6.92424 27.2803 10.7879 20.9924 15.75 15.8409C20.7121 10.6894 26.4886 6.76893 33.0795 4.07954C39.6705 1.39015 46.8106 0.0454502 54.5 0.0454502C61.0909 0.0454502 67.2273 1.01136 72.9091 2.94318C78.5909 4.83712 83.6288 7.52651 88.0227 11.0114C92.4545 14.4962 96.072 18.6439 98.875 23.4545C101.678 28.2273 103.477 33.4924 104.273 39.25H79.2727Z\" fill=\"url(#paint0_linear_113_123)\"/>"
           "<defs>"
           "<linearGradient id=\"paint0_linear_113_123\" x1=\"-8\" y1=\"60\" x2=\"114\" y2=\"60\" gradientUnits=\"userSpaceOnUse\">"
           "<stop stop-color=\"#FFBAFC\"/>"
           "<stop offset=\"1\" stop-color=\"#A0225E\"/>"
           "</linearGradient>"
           "</defs>"
           "</svg>"
         );
}

// Return a link of the logo as a anchor ref tag
/*String getLogoLink() {
  return String(
           "<a href=\"https://www.grefur.com\" target=\"_blank\" class='logo-wrapper'>"
           "<svg viewBox=\"0 0 106 120\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\">"
           "<path d=\"M79.2727 39.25C78.4773 36.4848 77.3599 34.0417 75.9205 31.9205C74.4811 29.7614 72.7197 27.9432 70.6364 26.4659C68.5909 24.9508 66.2424 23.7955 63.5909 23C60.9773 22.2045 58.0795 21.8068 54.8977 21.8068C48.9508 21.8068 43.7235 23.2841 39.2159 26.2386C34.7462 29.1932 31.2614 33.4924 28.7614 39.1364C26.2614 44.7424 25.0114 51.5985 25.0114 59.7045C25.0114 67.8106 26.2424 74.7045 28.7045 80.3864C31.1667 86.0682 34.6515 90.4053 39.1591 93.3977C43.6667 96.3523 48.9886 97.8295 55.125 97.8295C60.6932 97.8295 65.447 96.8447 69.3864 94.875C73.3636 92.8674 76.3939 90.0455 78.4773 86.4091C80.5985 82.7727 81.6591 78.4735 81.6591 73.5114L86.6591 74.25H56.6591V55.7273H105.352V70.3864C105.352 80.6136 103.193 89.4015 98.875 96.75C94.5568 104.061 88.6098 109.705 81.0341 113.682C73.4583 117.621 64.7841 119.591 55.0114 119.591C44.1023 119.591 34.5189 117.186 26.2614 112.375C18.0038 107.527 11.5644 100.652 6.94318 91.75C2.35985 82.8106 0.0681819 72.2045 0.0681819 59.9318C0.0681819 50.5 1.43182 42.0909 4.15909 34.7045C6.92424 27.2803 10.7879 20.9924 15.75 15.8409C20.7121 10.6894 26.4886 6.76893 33.0795 4.07954C39.6705 1.39015 46.8106 0.0454502 54.5 0.0454502C61.0909 0.0454502 67.2273 1.01136 72.9091 2.94318C78.5909 4.83712 83.6288 7.52651 88.0227 11.0114C92.4545 14.4962 96.072 18.6439 98.875 23.4545C101.678 28.2273 103.477 33.4924 104.273 39.25H79.2727Z\" fill=\"url(#paint0_linear_113_123)\"/>"
           "<defs>"
           "<linearGradient id=\"paint0_linear_113_123\" x1=\"-8\" y1=\"60\" x2=\"114\" y2=\"60\" gradientUnits=\"userSpaceOnUse\">"
           "<stop stop-color=\"#FFBAFC\"/>"
           "<stop offset=\"1\" stop-color=\"#A0225E\"/>"
           "</linearGradient>"
           "</defs>"
           "</svg>"
           "</a>"
         );
}*/

String getLogoLink() {
  return String(
           "<a href=\"https://www.grefur.com\" target=\"_blank\" class='logo-wrapper'>"
           "<svg viewBox=\"-2 -2 110 124\" fill=\"none\" xmlns=\"http://www.w3.org/2000/svg\" class='svgLogo'>"
           "<path d=\"M79.2727 39.25C78.4773 36.4848 77.3599 34.0417 75.9205 31.9205C74.4811 29.7614 72.7197 27.9432 70.6364 26.4659C68.5909 24.9508 66.2424 23.7955 63.5909 23C60.9773 22.2045 58.0795 21.8068 54.8977 21.8068C48.9508 21.8068 43.7235 23.2841 39.2159 26.2386C34.7462 29.1932 31.2614 33.4924 28.7614 39.1364C26.2614 44.7424 25.0114 51.5985 25.0114 59.7045C25.0114 67.8106 26.2424 74.7045 28.7045 80.3864C31.1667 86.0682 34.6515 90.4053 39.1591 93.3977C43.6667 96.3523 48.9886 97.8295 55.125 97.8295C60.6932 97.8295 65.447 96.8447 69.3864 94.875C73.3636 92.8674 76.3939 90.0455 78.4773 86.4091C80.5985 82.7727 81.6591 78.4735 81.6591 73.5114L86.6591 74.25H56.6591V55.7273H105.352V70.3864C105.352 80.6136 103.193 89.4015 98.875 96.75C94.5568 104.061 88.6098 109.705 81.0341 113.682C73.4583 117.621 64.7841 119.591 55.0114 119.591C44.1023 119.591 34.5189 117.186 26.2614 112.375C18.0038 107.527 11.5644 100.652 6.94318 91.75C2.35985 82.8106 0.0681819 72.2045 0.0681819 59.9318C0.0681819 50.5 1.43182 42.0909 4.15909 34.7045C6.92424 27.2803 10.7879 20.9924 15.75 15.8409C20.7121 10.6894 26.4886 6.76893 33.0795 4.07954C39.6705 1.39015 46.8106 0.0454502 54.5 0.0454502C61.0909 0.0454502 67.2273 1.01136 72.9091 2.94318C78.5909 4.83712 83.6288 7.52651 88.0227 11.0114C92.4545 14.4962 96.072 18.6439 98.875 23.4545C101.678 28.2273 103.477 33.4924 104.273 39.25H79.2727Z\" "
           "fill=\"url(#paint0_linear_113_123)\" stroke=\"white\" stroke-width=\"1.5\"/>"
           "<defs>"
           "<linearGradient id=\"paint0_linear_113_123\" x1=\"0\" y1=\"0\" x2=\"106\" y2=\"120\" gradientUnits=\"userSpaceOnUse\">"
           "<stop stop-color=\"#FFBAFC\"/>"
           "<stop offset=\"1\" stop-color=\"#A0225E\"/>"
           "</linearGradient>"
           "</defs>"
           "</svg>"
           "</a>"
         );
}

String getNavigationBar() {
  String html =
    "<style>"
    ".nav-container { "
    "  display: flex; justify-content: start; align-items: center; flex-wrap: wrap; "
    "  min-height: 80px; gap: 10px; width: 100%; overflow-x: hidden; "
    "  background-color: var(--primary); backdrop-filter: blur(20px); "
    "  -webkit-backdrop-filter: blur(20px); box-sizing: border-box; padding: 10px 20px; "
    "}"
    ".logo-wrapper { "
    "  height: 50px; " /* Fixed height for big screens */
    "  display: flex; align-items: center; justify-content: center; "
    "  fill: var(--white); margin-right: 20px; "
    "}"
    ".svgLogo { "
    "  height: 100%; width: auto; display: block; "
    "}"
    ".nav-button { "
    "  padding: 10px 24px; font-weight: 500; white-space: nowrap; font-size: 1rem; "
    "  text-decoration: none; background-color: white; color: var(--dark-text); "
    "  border: none; border-radius: 0; cursor: pointer; transition: all 0.1s ease; "
    "}"
    ".nav-button:hover { filter: brightness(1.05); }"
    ".nav-button.home-button { background-color: #e575ab; color: white; }"

    "@media (max-width: 600px) {"
    "  .nav-container { justify-content: center; padding: 15px 10px; gap: 8px; }"
    "  .logo-wrapper { "
    "    height: 40px; " /* Scaled down for mobile */
    "    width: 100%; margin: 0 0 10px 0; "
    "  }"
    "  .nav-button { "
    "    padding: 10px 12px; font-size: 0.9rem; flex: 1 1 40%; text-align: center; "
    "  }"
    "}"
    "</style>"
    "<nav class='nav-container'>";

  #if (!defined(ESP8266))
  html += getLogoLink();
  #endif
  String homeButton     = "<a href='/' class='nav-button home-button'>Home</a>";
  String firmwareButton = "<a href='/firmware' class='nav-button'>Firmware</a>";
  String certButton     = "<a href='/certificate' class='nav-button'>Certificates</a>";
  String backupButton   = "<a href='/config/backup' class='nav-button'>Backup</a>";

  html += homeButton + firmwareButton + certButton + backupButton;

  html += "</nav>";
  return html;
}


// Default header to use on all pages
String getHTMLHeader(const String& title) {
  return String("<html><head><meta charset='UTF-8'><title>") + title +
         "</title><meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no'>"
         "<style>"
         "body {font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0px; padding: 0px; border: 0px;}"
         "main {padding:16px;}"
         ":root{--primary: #a0225e;--secondary: #ffbafc;--dark-text: #4a0a28; --white: #ffffff;}"
         "h1 {color: var(--dark-text); font-weight: 600; font-size: 1.5rem;}"
         "h2 {color: var(--dark-text); font-weight: 600; }"
         "h3 {color: var(--dark-text); }"
         "form, .classAsForm {padding: 20px; border-bottom:1px solid #eee;}"
         "input, select {width: 100%; padding: 10px; margin: 8px 0 15px; border: 1px solid #ddd; border-radius: 0px; box-sizing: border-box; font-size: 16px; transition: all 0.3s ease;}"
         "input:focus, select:focus {border-color: #6c757d; outline: none; box-shadow: 0 0 0 2px rgba(108, 117, 125, 0.2);}"
         "input[type='submit'], .inheritSubmitStyle {"
         "  background-color: var(--primary); color: white; padding: 10px 20px; border: none; cursor: pointer;"
         "  font-weight: 600; transition: all 0.3s ease; box-shadow: 0 2px 5px rgba(0,0,0,0.1);"
         "}"
         "input[type='submit']:hover, .inheritSubmitStyle:hover { filter:invert(5%); }"
         "input[type='checkbox'] {width: auto; margin-right: 8px;}"
         "button { font-size: 1rem; padding: 10px 20px; cursor: pointer; transition: all 0.3s ease; }"
         ".sensor-table { width: 100%; border-collapse: collapse; }"
         ".sensor-table th, .sensor-table td { padding: 12px 8px; text-align: left; border-bottom: 1px solid #ddd; }"
         ".btn-edit { background-color: var(--primary); color: white; padding: 6px 12px; border: none; border-radius: 0px; cursor: pointer; text-decoration: none; display: inline-block; font-size: 14px; }"
         ".btn-delete { background-color: var(--dark-text); color: white; padding: 6px 12px; border: none; border-radius: 0px; cursor: pointer; display: inline-block; font-size: 14px; }"
         ".actions { display:flex; flex-direction:row; gap:8px; align-items: center; padding-left: 0px; background:inherit;}"
         ".info {background: #e7f3fe; border-left: 6px solid #2196F3; padding: 12px; margin: 15px 0; border-radius: 0 4px 4px 0;}"
         ".section {margin-bottom: 25px; border-bottom: 1px solid #eee; padding-bottom: 25px;}"
         ".grid {display: grid; grid-template-columns: 1fr 1fr; gap: 20px;}"
         ".sensorHeaderSmallScreen {display:none;}"
         "@media (max-width: 768px) {"
         "  .grid {grid-template-columns: 1fr; gap: 15px;}"
         "  .sensor-table, .sensor-table thead, .sensor-table tbody, .sensor-table th, .sensor-table td, .sensor-table tr { display: flex; flex-direction:column-reverse; }"
         "  .sensor-table thead { display: none; }"
         "  .sensor-table tr { margin-bottom: 15px; border: 1px solid #eee; border-radius: 4px; box-shadow: 0 2px 4px rgba(0,0,0,0.05); }"
         "  .sensor-table td { border: none; position: relative; border-bottom: 1px solid #f9f9f9; display: flex; align-items: start; }"
         "  .sensor-table td:before { content: attr(data-label); position: absolute; left: 10px; width: 40%; font-weight: bold; color: var(--primary); font-size: 13px; }"
         "  .sensor-table td.actions { display:flex; flex-direction:row; gap:8px; align-items: center; padding-left: 10px; background: #fafafa;}"
         "  .sensor-table td.actions:before { content: ''; }"
         "  .sensorHeaderSmallScreen { font-weight: 700; color: var(--primary);display:block;margin:4px 0px; }"
         "}"
         "</style>"
         "<link rel='icon' href='/favicon.svg' type='image/svg+xml'>"
         "</head>"
         "<body>" +
         getNavigationBar();
}

String mainBody(String markUpPosistion){
  if (markUpPosistion == "start"){
    return "<main>";
  } else if (markUpPosistion == "end"){
    return "</main>";
  } else {
    return "";
  }
}

// Returns a generic JavaScript snippet for AJAX submission
String getAjaxFormScript() {
  String script;

  script += "<script>";
  script += "document.addEventListener('DOMContentLoaded', function() {";
  script += "  document.querySelectorAll('form').forEach(form => {";
  script += "    form.addEventListener('submit', function(e) {";
  script += "      e.preventDefault();";
  script += "      const formData = new FormData(form);";
  script += "      const action = form.getAttribute('action');";
  script += "      const responseEl = form.querySelector('#responseMessage');"; // find responseMessage inside this form
  script += "      if (responseEl) responseEl.innerHTML = '<p>Saving...</p>';"; // feedback during fetch
  script += "      fetch(action, { method: 'POST', body: formData })";
  script += "        .then(response => response.text())";
  script += "        .then(data => {";
  script += "          if (responseEl) responseEl.innerHTML = data;";
  script += "        })";
  script += "        .catch(err => {";
  script += "          console.error('AJAX error:', err);";
  script += "          if (responseEl) responseEl.innerHTML = '<p style=\"color:red;\">Error submitting form</p>';";
  script += "        });";
  script += "    });";
  script += "  });";
  script += "});";
  script += "</script>";

  return script;
}


// Default footer to use on all pages
String getHTMLFooter() {
  String html;

  html += "<footer style='margin-top: 20px; text-align: center; color: #777;'>"
          "Grefur Sensor v" + getFirmwareVersion() + "<br>"
          "All rights reserved &copy; Grefur"
          "</footer></body></html>";


  return html;
}

String getDeviceIdParagraph(){
  String html;
  #if defined(ESP8266)
  html += "<p><strong>Device ID:</strong> " + String(ESP.getChipId(), HEX) + "</p>";
  return html;
  #elif defined(ESP32)
  uint64_t chipId = ESP.getEfuseMac();
  uint32_t shortId = (uint32_t)(chipId >> 32);
  html += "<p><strong>Device ID:</strong> " + String(shortId, HEX) + "</p>";
  return html;
  #endif
}

String getDeviceInfoHTML() {
  String html;
  html += "<div class='info'>";
  html += getDeviceIdParagraph();
  html += "<p><strong>Firmware Version:</strong> " + getFirmwareVersion() + "</p>";

  if (WiFi.getMode() == WIFI_AP) {
    html += "<p><strong>Mode:</strong> Configuration (AP)</p>";
    html += "<p><strong>AP Name:</strong> " + generateAPName() + "</p>";
  } else {

    String operationMode;
    if (!mqttClient.connected()) {
      operationMode = "No broker connected";
    } else {
      operationMode = "Normal operation";
    }

    html += "<p><strong>Mode: </strong>" + operationMode + "</p>";
    html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
    html += "<p><strong>WiFi RSSI:</strong> " + String(WiFi.RSSI()) + " dBm</p>";
    //html += "<p><strong>MQTT Status:</strong> " + getMQTTStatus() + "</p>";
  }

  html += "</div>";
  return html;
}

String getSimpleDeviceInfoHTML(String measurement = "") {
  String html;
  html += "<div class='info'>";
  html += getDeviceIdParagraph();
  html += "<p><strong>Firmware Version:</strong> " + getFirmwareVersion() + "</p>";

  if (measurement != "") {
    html += "<p><strong>Last Measurement:</strong> " + measurement + "</p>";
  }

  html += "</div>";
  return html;
}


String getLEDControls() {
  String html = "<button class='inheritSubmitStyle' onclick=\"controlLED('on')\" title='LED will wink'>Wink On</button>";
  html += "<button class='inheritSubmitStyle' onclick=\"controlLED('off')\" title='LED will wink'>Wink Off</button>";
  return html;
}

String getLedControlHTML() {
  String html;
  // Rettet manglende ' etter gap:4px
  html += "<div class='section classAsForm' style='display:flex; flex-direction:column;gap:12px;'>";
  html += "<h2>LED Control</h2>";
  html += getLEDControls();

  html += "<script>";
  html += "function controlLED(state) {";
  html += "  fetch('/config/wink?state=' + state)";
  html += "    .then(response => response.text());";
  html += "}";
  html += "</script>";

  html += "</div>";
  return html;
}




String getDeviceInformationForm() {
  String html;

  // Start form
  html += "<form id='deviceForm' action='/config/device' method='POST'>";

  // Device Info Section
  html += "<div class='section'>";
  html += "<h2>Device Information</h2>";

  // Device Name input field
  html += "Device Name: <input type='text' name='deviceName' value='" + String(config.device.deviceName) + "'><br>";

  // Location input field
  html += "Location: <input type='text' name='deviceLocation' value='" + String(config.device.location) + "'><br>";

  // Close section div
  html += "</div>";

  // Submit button
  html += "<input type='submit' value='Save Device Info'><br>";

  // Placeholder for server response
  html += "<div id='responseMessage'></div>";  // This is where the <p> snippet will be inserted

  // Close form
  html += "</form><br>";

  return html;
}


String getWiFiConfigForm() {
  String html;
  if (WIRELESS_NETWORK) {
    html += "<form action='/config/wifi' method='POST'>";
    html += "<h2>WiFi Configuration</h2>";
    // Scan for networks
    /*int n = WiFi.scanNetworks();
                                                                if (n > 0) {
                                                                  html += "<label>Available networks:</label><br>";
                                                                  html += "<select onchange=\"this.form.ssid.value=this.value\" style='margin-bottom:8px;width:100%;'>";
                                                                  html += "<option value=''>-- Select network --</option>";
                                                                  for (int i = 0; i < n; i++) {
                                                                    String ssid = WiFi.SSID(i);
                                                                    int rssi = WiFi.RSSI(i);
                                                                    bool encrypted = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
                                                                    html += "<option value='" + ssid + "'>";
                                                                    html += ssid;
                                                                    html += " (" + String(rssi) + " dBm)";
                                                                    html += encrypted ? " [secured]" : " [open]";
                                                                    html += "</option>";
                                                                  }
                                                                  html += "</select><br>";
                                                                  WiFi.scanDelete();
                                                                } else {
                                                                  html += "<small>No networks found.</small><br>";
                                                                }*/
    // SSID
    html += "SSID: <input type='text' name='ssid' value='" + String(config.network.wifiSSID) + "'><br>";
    // Password
    html += "Password: <input type='password' name='password' placeholder='";
    if (strlen(config.network.wifiPassword) > 0) {
      html += "[Password is set]";
    } else {
      html += "Enter new password";
    }
    html += "'><br>";
    html += "<input type='submit' value='Save WiFi'>";
  } else {
    html += "<form action='/config/ethernet' method='POST'>";
    html += "<h2>Ethernet Configuration</h2>";
    html += "Use DHCP: <input type='checkbox' name='useDHCP' ";
    if (config.network.useDHCP) html += "checked";
    html += "><br>";
    html += "Static IP: <input type='text' name='staticIP' value='" + config.network.staticIP.toString() + "'><br>";
    html += "Gateway: <input type='text' name='gateway' value='" + config.network.gateway.toString() + "'><br>";
    html += "Subnet: <input type='text' name='subnet' value='" + config.network.subnet.toString() + "'><br>";
    html += "<input type='submit' value='Save Ethernet'>";
  }
  html += "<div id='responseMessage'></div>";
  html += "</form><br>";
  return html;
}


/* Summary of function: Generates HTML for MQTT settings. Locks fields if Grefur Services are active. */
String DisplayMQTTSettingsHTML() {
  String html;
  // Verify if Grefur Services is registered as used. Make fields grey and disabled
  String locked = config.mqtt.useGrefurBackend ? " readonly style='background-color: #f0f0f0; color: #666;'" : "";
  String disabled = config.mqtt.useGrefurBackend ? " disabled" : "";

  html += "<form id='mqttConfig' method='POST' action='/config/mqtt'>";
  html += "<h2 id='mqttSettings'>MQTT Settings</h2>";

  // Grefur Services Toggle
  html += "<div style='background-color: color-mix(in srgb, var(--primary), transparent 50%); padding: 15px; border-radius: 8px; border: 1px solid var(--primary); margin-bottom: 20px;'>";
  html += "  <label style='font-weight: bold; color: var(--dark-text);'>";
  html += "    <input type='checkbox' name='useGrefur' " + String(config.mqtt.useGrefurBackend ? "checked" : "") + "> Use Grefur Services";
  html += "  </label>";
  html += "  <small style='display:block; color: var(--dark-text);'>When enabled, credentials and broker settings are managed automatically by Grefur Backend.</small>";
  html += "</div>";

  html += "<div class='grid'>";

  // Left Column
  html += "<div>";
  html += "<label for='mqttServer'>Broker URL:</label><br>";
  html += "<input id='mqttServer' type='text' name='mqttServer' value='" + String(config.mqtt.brokerURL) + "'" + locked + "><br>";

  html += "<label for='mqttPort'>Port:</label><br>";
  uint16_t port = (config.mqtt.port > 0) ? config.mqtt.port : 1883;
  html += "<input id='mqttPort' type='number' name='mqttPort' value='" + String(port) + "' min='1' max='65535'" + locked + "><br>";

  html += "<label for='mqttClientId'>Client ID:</label><br>";
  html += "<input id='mqttClientId' type='text' name='mqttClientId' value='" + String(config.mqtt.clientID) + "'" + locked + "><br>";
  html += "</div>";

  // Right Column
  html += "<div>";
  html += "<label for='mqttUser'>Username:</label><br>";
  html += "<input id='mqttUser' type='text' name='mqttUser' value='" + String(config.mqtt.username) + "'" + locked + "><br>";

  html += "<label for='mqttPassword'>Password:</label><br>";
  html += "<input id='mqttPassword' autocomplete='off' type='password' name='mqttPassword'" + locked + " placeholder='";
  if (strlen(config.mqtt.password) > 0) {
    html += "[Password is set]";
  } else {
    html += "Enter new password";
  }
  html += "'><br>";

  // TOPIC BASE - This remains UNLOCKED even if Grefur is active
  html += "<label for='mqttTopic'>Topic base:</label><br>";
  html += "<input id='mqttTopic' type='text' name='mqttTopic' value='" + String(config.mqtt.baseTopic) + "' placeholder='e.g. grefur/sensors'><br>";
  html += "</div>";

  html += "</div>"; // Close grid

  // Settings
  html += "<label for='mqttQos'>QoS:</label><br>";
  html += "<select id='mqttQos' name='mqttQos'" + disabled + ">";
  html += "<option value='0'" + String(config.mqtt.qos == 0 ? " selected" : "") + ">0 - At most once</option>";
  html += "<option value='1'" + String(config.mqtt.qos == 1 ? " selected" : "") + ">1 - At least once</option>";
  html += "<option value='2'" + String(config.mqtt.qos == 2 ? " selected" : "") + ">2 - Exactly once</option>";
  html += "</select><br>";

  html += "<label><input type='checkbox' name='mqttRetain'" + String(config.mqtt.retain ? " checked" : "") + disabled + "> Retain Messages</label>";
  html += "<small style='display:block; margin-top: -5px; margin-bottom: 10px; color: #666;'>If disabled, clear topic for status in broker to remove existing retained messages</small><br>";

  // Message Format Section
  html += "<h3>Message Format</h3>";
  html += "<label for='useNestedJson'>Message Format:</label><br>";
  html += "<select id='useNestedJson' name='useNestedJson'" + disabled + ">";
  html += "<option value='1'" + String(config.mqtt.useNestedJson ? " selected" : "") + ">Nested JSON</option>";
  html += "<option value='0'" + String(!config.mqtt.useNestedJson ? " selected" : "") + ">Flat Topics</option>";
  html += "</select><br><br>";

  html += "<input type='submit' value='Save MQTT Settings' style='width: 100%; padding: 10px; cursor: pointer;'>";
  html += "<div id='responseMessage'></div>";

  html += "</form>";

  return html;
}



String DisplayOperationSettings() {
  String html;
  bool isLocked = config.mqtt.useGrefurBackend;

  html += "<form method='POST' action='/config/operations'>";
  html += "<div class='section'>";

  html += "<div class='grid-container'>"; // Changed class to grid-container

  // Left column (50%)
  html += "<div class='grid-left '>";
  html += "<h2 id='operationSettings'>Operation Settings</h2>";

  /* Summary of code: min-attribute to enforce a 100ms lower limit in the UI. */
  html += "<div style='margin-bottom: 1em;'>";
  html += "<label for='publishInterval' style='display: block;'>Publish Interval (ms):</label>";
  html += "<input id='publishInterval' type='number' name='publishInterval' min='100' step='50' value='" + String(config.operation.publishInterval) + "' style='max-width: 350px; width: 100%;' required>";
  html += "<p style='font-size: 0.8em; color: #666; margin-top: 4px;'>Minimum interval is 100ms - individual sensor intervals cannot publish faster than this. </p>";
  html += "</div>";

  // Config Password input group
  html += "<div style='margin-bottom: 1em;'>";
  html += "<label for='configPassword' style='display: block;'>Config Password:</label>";
  html += "<input id='configPassword' autocomplete='off' type='password' name='configPassword' placeholder='Enter new password' style='max-width: 350px; width: 100%;'>";
  html += "</div>";

  html += "</div>"; // Close grid-left

  // Right column (50%)
  html += "<div class='grid-right'>";
  html += "<h2 id='publishOptions'>Publish Options</h2>";
  html += "<div class='input-group'>";
  html += "<label><input type='checkbox' name='publishTemperature'" + String(config.operation.publishMeasuredValues ? " checked" : "") + "> Measured values</label>";
  html += "<label><input type='checkbox' name='publishIP'" + String(config.operation.publishIP ? " checked" : "") + "> IP Address</label>";
  if (isLocked) {
    html += "<label style='color: #888;'><input type='checkbox' checked disabled> Status <small>(Required by Grefur)</small></label>";
    html += "<input type='hidden' name='publishStatus' value='true'>";
  } else {
    html += "<label><input type='checkbox' name='publishStatus'" + String(config.operation.publishStatus ? " checked" : "") + "> Status</label>";
  }

  // Device Info - Locked if Grefur Backend is used
  if (isLocked) {
    html += "<label style='color: #888;'><input type='checkbox' checked disabled> Device Info <small>(Required by Grefur)</small></label>";
    html += "<input type='hidden' name='publishDeviceInfo' value='true'>";
  } else {
    html += "<label><input type='checkbox' name='publishDeviceInfo'" + String(config.operation.publishDeviceInfo ? " checked" : "") + "> Device Info</label>";
  }
  html += "<label><input type='checkbox' name='publishRSSI'" + String(config.operation.publishRSSI ? " checked" : "") + "> RSSI (with status)</label>";
  html += "</div>"; // Close checkbox-group
  html += "</div>"; // Close grid-right

  html += "</div>";
  html += "<br>";

  html += "<div class='form-footer'>";
  html += "<input type='submit' value='Save Operation Settings'>";
  html += "<div id='responseMessage'></div>";  // This is where the <p> snippet will be inserted
  html += "</div>";
  html += "</div>"; // Close section
  html += "</form>";

  // Add CSS for the grid layout
  html += "<style>";
  html += ".grid-container { display: flex; gap: 20px; }";
  html += ".grid-left, .grid-right { flex: 1; flex-direction: column; gap: 8px; }"; // Each takes 50% of space
  html += ".input-group { display: flex; flex-direction: column; gap: 8px; }";
  html += "</style>";

  return html;
}

String SensorTableScript() {
  String html;

  html += "<script id='updateSensorTableScript'>";

  // --- Reusable helper function for updating the table ---
  html += "async function updateSensorTable(url, options) {";
  html += "  try {";
  html += "    const response = await fetch(url, options);";
  html += "    if (!response.ok) throw new Error('Server error: ' + response.status);";
  html += "    const text = await response.text();";
  html += "    const parser = new DOMParser();";
  html += "    const doc = parser.parseFromString(text, 'text/html');";
  html += "    const newTable = doc.querySelector('#sensor-table');";
  html += "    const oldTable = document.getElementById('sensor-table');";
  html += "    if (oldTable && newTable) oldTable.replaceWith(newTable);";
  html += "    else console.warn('Could not find table to replace');";
  html += "  } catch (err) {";
  html += "    console.error('Failed to update sensor table:', err);";
  html += "    alert('Failed to update sensor table: ' + err.message);";
  html += "  }";
  html += "}";

  // --- Form submission handler using a clean button ---
  html += "document.getElementById('addSensorBtn').addEventListener('click', async function() {";
  html += "  const form = document.getElementById('sensorForm');";
  html += "  const nameField = form.querySelector('[name=name]');";
  html += "  const typeField = form.querySelector('[name=type]');";
  html += "  if (!nameField.value.trim() || !typeField.value.trim()) {";
  html += "    alert('Please enter both sensor name and type.');";
  html += "    return;";  // Stop execution if either is empty
  html += "  }";
  html += "  const submitButton = this;";
  html += "  submitButton.disabled = true;";
  html += "  const formData = new FormData(form);";
  html += "  await updateSensorTable(form.action, { method: form.method, body: formData });";
  html += "  form.reset();";
  html += "  const dynamicContainer = document.getElementById('dynamicSensorFields');";
  html += "  if (dynamicContainer) dynamicContainer.innerHTML = '';";
  html += "  submitButton.disabled = false;";
  html += "});";



  // --- Delete handler using the reusable function ---
  html += "function deleteSensor(id) {";
  html += "  if (confirm('Are you sure you want to delete sensor ' + id + '?')) {";
  html += "    updateSensorTable('/config/sensor?id=' + id + '&responseFormat=html', { method: 'DELETE' });";
  html += "  }";
  html += "}";

  html += "</script>";

  return html;
}




// One row for each sensor with all the informationfields, and a button for edit and one for delete. Make these dummy for so long
String DisplaySensorsInProduction() {
  String html;
  bool hasConfiguredSensors = false;

  html += "<div class='sensor-list' id='sensor-list'>";
  html += "<h2 id='configuredSensors'>Configured Sensors</h2>";
  html += "<table class='sensor-table' id='sensor-table'>";
  html += "<thead><tr>";
  html += "<th>ID</th>";
  html += "<th>Name</th>";
  html += "<th>Type</th>";
  html += "<th>Details</th>";
  html += "<th>Actions</th>";
  html += "</tr></thead>";
  html += "<tbody>";

  // Loop through all possible sensors
  for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
    Serial.println("Sensortype:" + config.sensors[i].type);
    if (config.sensors[i].type == SENSOR_UNCONFIGURED) {
      continue;
    }

    hasConfiguredSensors = true;

    html += "<tr>";
    html += "<td><p class='sensorHeaderSmallScreen'>Sensor ID </p>" + String(i) + "</td>";
    html += "<td><p class='sensorHeaderSmallScreen'>Sensor Name </p>" + String(config.sensors[i].name) + "</td>";

    // Convert type code to readable name
    String typeName = "Unknown";
    if (config.sensors[i].type == SENSOR_NTC) typeName = "NTC";
    else if (config.sensors[i].type == SENSOR_ANALOG) typeName = "Analog";
    else if (config.sensors[i].type == SENSOR_DIGITAL) typeName = "Digital";

    html += "<td><p class='sensorHeaderSmallScreen'>Type </p>" + typeName + "</td>";

    // Get sensor details
    String details = "";

    String publishMode;
    switch (config.sensors[i].publishPolicy.trigger) {
      case PUBLISH_ON_THRESHOLD_OR_INTERVAL: publishMode = "Publish: Threshold or Interval"; break;
      case PUBLISH_ON_CHANGE_OF_THRESHOLD:   publishMode = "Publish: Changes only";          break;
      case PUBLISH_ON_INTERVAL:              publishMode = "Publish: Interval only";          break;
      default:                               publishMode = "Publish: Unknown";                break;
    }

    if (config.sensors[i].type == SENSOR_NTC) {
      details = "Pin A" + String(config.sensors[i].ntc.analogPin) + ", " +
                String(config.sensors[i].ntc.nominalResistance) + "Ω @ " +
                String(config.sensors[i].ntc.nominalTemp) + "°C, B=" +
                String(config.sensors[i].ntc.bCoefficient) + ", " +
                String(config.sensors[i].ntc.unit) + " " +
                publishMode;
    }
    else if (config.sensors[i].type == SENSOR_ANALOG) {
      details = "GPIO" + String(config.sensors[i].analog.analogPin) + ", " +
                String(config.sensors[i].analog.inputMin) + "-" +
                String(config.sensors[i].analog.inputMax) + "V → " +
                String(config.sensors[i].analog.scaleMin) + "-" +
                String(config.sensors[i].analog.scaleMax) + " " +
                String(config.sensors[i].analog.scaleUnit) + " " +
                publishMode;
    }
    else if (config.sensors[i].type == SENSOR_DIGITAL) {
      details = "GPIO" + String(config.sensors[i].digital.digitalPin) +
                (config.sensors[i].digital.inverted ? " (Inverted)" : "") + " " +
                publishMode;
    }


    html += "<td>" + details + "</td>";
    html += "<td class='actions'>";
    html += "<a class='btn-edit' href='/config/sensor/edit?id=" + String(i) + "'>Edit</a>";
    html += "<button class='btn-delete' onclick='deleteSensor(" + String(i) + ")'>Delete</button>";
    html += "</td>";
    html += "</tr>";
  }
  html += "</tbody>";
  html += "</table>";
  html += "</div>";


  return html;
}


String SensorTypeFieldsScript(signed int sensorType = -1) {
  String html;

  // Reusable publish policy fields (NTC + Analog — threshold is meaningful)
  String publishPolicyFields =
    "container.innerHTML += \"<div class='form-group'><label>Publish mode:</label>"
    "<select name='publishTrigger' id='publishTrigger' onchange='toggleThreshold()'>"
    "<option value='0'>Threshold or Interval</option>"
    "<option value='1'>Changes only (threshold)</option>"
    "<option value='2'>Interval only</option>"
    "</select></div>\";"

    "container.innerHTML += \"<div class='form-group' id='thresholdGroup'>"
    "<label>Change Threshold:</label>"
    "<input type='number' step='any' name='changeThreshold' value='0.5'>"
    "<small>Minimum change required to trigger publish</small>"
    "</div>\";"

    "container.innerHTML += \"<div class='form-group'>"        // <-- ny
    "<label>Publish Interval (ms):</label>"
    "<input type='number' name='intervalMs' value='60000'>"
    "<small>Sensor publish interval. Cannot be faster than the global publish interval.</small>"
    "</div>\";"

    "function toggleThreshold(){"
    "  const t = document.getElementById('publishTrigger');"
    "  const g = document.getElementById('thresholdGroup');"
    "  if(!t||!g) return;"
    "  const show = (t.value == '0' || t.value == '1');"
    "  g.style.display = show ? 'block' : 'none';"
    "  document.querySelector('[name=changeThreshold]').disabled = !show;"
    "}"
    "toggleThreshold();";

  String publishPolicyFieldsDigital =
    "container.innerHTML += \"<div class='form-group'><label>Publish mode:</label>"
    "<select name='publishTrigger'>"
    "<option value='0'>Threshold or Interval</option>"
    "<option value='2'>Interval only</option>"
    "</select></div>\";"

    "container.innerHTML += \"<div class='form-group'>"        // <-- ny
    "<label>Publish Interval (ms):</label>"
    "<input type='number' name='intervalMs' value='60000'>"
    "<small>Sensor publish interval. Cannot be faster than the global publish interval.</small>"
    "</div>\";";

  // --- NTC sensor fields ---
  String ntc;
  ntc += "container.innerHTML += \"<div class='form-group'><label>Analog Pin:</label><input type='number' name='analogPin' required></div>\";";
  ntc += publishPolicyFields;
  ntc += "container.innerHTML += \"<div class='form-group'><label>Temperature Unit:</label><input type='text' name='unit' required></div>\";";
  ntc += "container.innerHTML += \"<div class='form-group'>"
         "<label>Thermistor type:</label>"
         "<select name='thermistorType' id='thermistorType'>"
         "<option value='ntc'>NTC</option>"
         "<option value='ptc'>PTC</option>"
         "</select></div>\";";
  ntc += "container.innerHTML += \"<div class='form-group'>"
         "<label>Preset:</label>"
         "<select id='thermistorPreset'>"
         "<option value='custom'>Custom</option>"
         "<option value='ntc10k'>NTC 10k B3950</option>"
         "<option value='ntc100k'>NTC 100k B3950</option>"
         "<option value='ptc1k'>PTC Pt1000</option>"
         "<option value='ptc2k'>PTC KTY81-210</option>"
         "</select></div>\";";
  ntc += "container.innerHTML += \"<div class='form-group'><label>Series Resistor:</label>"
         "<input id='seriesResistor' type='number' step='any' name='seriesResistor' required></div>\";";
  ntc += "container.innerHTML += \"<div class='form-group'><label>Nominal Resistance:</label>"
         "<input id='nominalResistance' type='number' step='any' name='nominalResistance' required></div>\";";
  ntc += "container.innerHTML += \"<div class='form-group'><label>Nominal Temp (°C):</label>"
         "<input id='nominalTemp' type='number' step='any' name='nominalTemp' required></div>\";";
  ntc += "container.innerHTML += \"<div class='form-group'><label>B Coefficient:</label>"
         "<input id='bCoefficient' type='number' step='any' name='bCoefficient' required></div>\";";
  ntc += "const presetEl = document.getElementById('thermistorPreset');"
         "if(presetEl){"
         "  presetEl.onchange = function(){"
         "    const preset = this.value;"
         "    const series = document.getElementById('seriesResistor');"
         "    const nominal = document.getElementById('nominalResistance');"
         "    const temp = document.getElementById('nominalTemp');"
         "    const beta = document.getElementById('bCoefficient');"
         "    if(!series||!nominal||!temp||!beta)return;"
         "    if(preset==='ntc10k'){ series.value=10000; nominal.value=10000; temp.value=25; beta.value=3950; }"
         "    else if(preset==='ntc100k'){ series.value=100000; nominal.value=100000; temp.value=25; beta.value=3950; }"
         "    else if(preset==='ptc1k'){ series.value=1000; nominal.value=1000; temp.value=25; beta.value=''; }"
         "    else if(preset==='ptc2k'){ series.value=2000; nominal.value=2000; temp.value=25; beta.value=''; }"
         "  };"
         "}";

  // --- Analog sensor fields ---
  String analog;
  analog += "container.innerHTML += \"<div class='form-group'><label>Analog Pin:</label><input type='number' name='analogPin' required></div>\";";
  analog += "container.innerHTML += \"<div class='form-group'><label>Input Min:</label><input type='number' step='any' name='inputMin' required></div>\";";
  analog += "container.innerHTML += \"<div class='form-group'><label>Input Max:</label><input type='number' step='any' name='inputMax' required></div>\";";
  analog += "container.innerHTML += \"<div class='form-group'><label>Scale Min:</label><input type='number' step='any' name='scaleMin' required></div>\";";
  analog += "container.innerHTML += \"<div class='form-group'><label>Scale Max:</label><input type='number' step='any' name='scaleMax' required></div>\";";
  analog += "container.innerHTML += \"<div class='form-group'><label>Scaled unit:</label><input type='text' name='scaleUnit' required></div>\";";
  analog += publishPolicyFields;

  // --- Digital sensor fields ---
  String digital;
  #if defined(ESP8266)
  digital += "container.innerHTML += \"<div class='form-group'><label>Digital Pin:</label><select name='digitalPin' required>"
             "<option value='16'>D0 (GPIO16)</option>"
             "<option value='5'>D1 (GPIO5)</option>"
             "<option value='4'>D2 (GPIO4)</option>"
             "<option value='14'>D5 (GPIO14)</option>"
             "<option value='12'>D6 (GPIO12)</option>"
             "<option value='13'>D7 (GPIO13)</option>"
             "</select></div>\";";
  #elif defined(ESP32)
  digital += "container.innerHTML += \"<div class='form-group'><label>Digital Pin:</label><select name='digitalPin' required>"
             "<option value='0'>GPIO0 (BOOT)</option>"
             "<option value='1'>GPIO1</option>"
             "<option value='2'>GPIO2</option>"
             "<option value='3'>GPIO3</option>"
             "<option value='4'>GPIO4</option>"
             "<option value='5'>GPIO5</option>"
             "</select></div>\";";
  #endif
  digital += "container.innerHTML += \"<div class='form-group'><label>Inverted:</label><input type='checkbox' name='inverted'></div>\";";
  digital += publishPolicyFieldsDigital;

  // If sensorType is specified, return just those fields (used when populating edit form)
  if (sensorType != -1) {
    if (sensorType == SENSOR_NTC)     return ntc;
    if (sensorType == SENSOR_ANALOG)  return analog;
    if (sensorType == SENSOR_DIGITAL) return digital;
    return "";
  }

  Serial.println("Rendrer dynamiske felter...");

  // --- JavaScript for dynamic fields ---
  html += "<script>";
  html += "document.getElementById('sensorType').addEventListener('change', function() {";
  html += "  const type = this.value;";
  html += "  const container = document.getElementById('dynamicSensorFields');";
  html += "  container.innerHTML = '';";
  html += "  console.log('Selected sensor type:', type);";
  html += "  if (type === 'ntc') {";
  html += "    " + ntc;
  html += "  } else if (type === 'analog') {";
  html += "    " + analog;
  html += "  } else if (type === 'digital') {";
  html += "    " + digital;
  html += "  }";
  html += "});";
  html += "</script>";

  return html;
}




String DisplayAddSensorFields() {
  String html = "";

  html += "<div class='sensor-form'>";
  html += "<h2 id='addNewSensor'>Add New Sensor</h2>";

  int availableSlots = MAX_SENSORS_ALLOWED - sensorManager.getConfiguredSensorCount();
  if (DEBUG) {
    Serial.println("Available slots: " + String(availableSlots));
  }

  html += "<div class='slots-info'>Available slots: " + String(availableSlots) + "</div>";

  if (availableSlots <= 0) {
    html += "<div class='no-slots'>No available sensor slots. Please delete a sensor to add a new one.</div>";
    html += "</div>"; // sensor-form
    return html;
  }

  html += "<form id='sensorForm' action='/config/sensor' method='POST'>";

  // Sensor type dropdown
  html += "<div class='form-group'>";
  html += "<label for='sensorType'>Sensor Type:</label>";
  html += "<select id='sensorType' name='type' required>";
  html += "<option value=''>Select a type</option>";

  String typesJson = sensorManager.retrieveSensorTypes();
  DynamicJsonDocument doc(512);
  // returning values = ntc, anlog, digital
  DeserializationError error = deserializeJson(doc, typesJson);
  if (!error && doc.containsKey("types")) {
    for (JsonVariant typeVar : doc["types"].as<JsonArray>()) {
      String type = typeVar.as<String>();
      String typeLower = type;
      typeLower.toLowerCase();
      html += "<option value='" + typeLower + "'>" + type + "</option>";
    }
  } else {
    html += "<option value=''>Error loading types</option>";
  }

  html += "</select></div>";

  // Sensor name
  html += "<div class='form-group'>";
  html += "<label for='name'>Sensor Name:</label>";
  html += "<input type='text' id='name' name='name' required>";
  html += "</div>";


  // Placeholder for dynamic fields
  html += "<div id='dynamicSensorFields'></div>";

  html += "<input type='button' id='addSensorBtn' class='inheritSubmitStyle' value='Add Sensor'>";
  html += "</form>";



  // Basic CSS
  html += "<style>";
  html += ".form-group { margin-bottom: 15px; }";
  html += ".form-group label { display: inline-block; width: 150px; white-space: nowrap; }";
  html += ".form-group input[type='checkbox'] { transform: scale(1.5); margin-right: 10px; margin-left: 10px; }";
  html += ".no-slots { color: red; font-weight: bold; margin: 20px 0; }";
  html += ".slots-info { margin-bottom: 15px; font-weight: bold; }";
  html += "</style>";

  html += "</div>"; // sensor-form

  return html;
}

/* Summary of function: Updated JS handler to support conditional validation and publishValue checkbox state */
String calculationTableScript() {
  String html;
  html += "<script id='updateCalcTableScript'>";

  // Reusable helper for AJAX updates
  html += "async function updateCalcTable(url, options) {";
  html += "  try {";
  html += "    const response = await fetch(url, options);";
  html += "    if (!response.ok) throw new Error('Server error: ' + response.status);";
  html += "    const text = await response.text();";
  html += "    const parser = new DOMParser();";
  html += "    const doc = parser.parseFromString(text, 'text/html');";
  html += "    const newTable = doc.querySelector('#calculation-table');";
  html += "    const oldTable = document.getElementById('calculation-table');";
  html += "    if (oldTable && newTable) oldTable.replaceWith(newTable);";
  html += "  } catch (err) { alert('Failed to update: ' + err.message); }";
  html += "}";

  // Click handler for Add Button
  html += "document.addEventListener('click', async function(e) {";
  html += "  if (e.target && e.target.id === 'addCalcBtn') {";
  html += "    const form = document.getElementById('calcForm');";
  html += "    const type = document.getElementById('calcType').value;";
  html += "    const name = form.querySelector('[name=name]').value.trim();";
  html += "    const expr = form.querySelector('[name=expression]').value.trim();";
  html += "    const tag = form.querySelector('[name=tag]').value.trim();";

  // Validering basert på type
  html += "    let isValid = !!name;";
  html += "    if (type === '3') { if(!tag) isValid = false; }";
  html += "    else { if(!expr) isValid = false; }";

  html += "    if (!isValid) { alert('Please fill in Name and ' + (type==='3'?'Binding Tag':'Expression')); return; }";

  html += "    e.target.disabled = true;";
  html += "    const formData = new FormData(form);";

  // Sørg for at publishValue sendes som 0 hvis den ikke er krysset av
  html += "    if (!form.querySelector('[name=publishValue]').checked) {";
  html += "      formData.set('publishValue', '0');";
  html += "    } else { formData.set('publishValue', '1'); }";

  html += "    await updateCalcTable(form.action + '?responseFormat=html', { method: form.method, body: formData });";

  html += "    form.reset();";
  html += "    e.target.disabled = false;";
  html += "    if(typeof toggleCalcFields === 'function') toggleCalcFields();";
  html += "  }";
  html += "});";

  html += "function deleteCalculation(id) {";
  html += "  if (confirm('Delete calculation slot ' + id + '?')) {";
  html += "    updateCalcTable('/config/calc/delete?id=' + id + '&responseFormat=html', { method: 'GET' });";
  html += "  }";
  html += "}";

  html += "</script>";
  return html;
}

/* Summary of function: Displays configured calculation slots with type-specific details and publish status */
String displayCalculationsInProduction() {
  String html;
  html += "<div class='sensor-list' id='calculation-list'>";
  html += "<h2 id='configuredCalculations'>Configured Calculations</h2>";
  html += "<table class='sensor-table' id='calculation-table'>";
  html += "<thead><tr><th>ID</th><th>Name</th><th>Type</th><th>Details</th><th>Publish</th><th>Actions</th></tr></thead>";
  html += "<tbody>";

  for (int i = 0; i < MAX_CALCULATIONS_ALLOWED; i++) {
    if (config.calculations[i].type <= 0) continue;
    auto& cfg = config.calculations[i];

    html += "<tr>";
    html += "<td><p class='sensorHeaderSmallScreen'>ID</p>" + String(i) + "</td>";
    html += "<td><p class='sensorHeaderSmallScreen'>Name</p><strong>" + String(cfg.name) + "</strong></td>";

    // Bestem type-navn og detaljer (Expression vs Tag)
    String typeName;
    String details;

    if (cfg.type == 1) { // CALC_EXPRESSION
      typeName = "Expression";
      details = "<code>" + String(cfg.expression.expression) + "</code>";
    } else if (cfg.type == 2) { // CALC_CONTINUOUS
      typeName = "Continuous";
      details = "<code>" + String(cfg.continuous.expression) + "</code>";
    } else if (cfg.type == 3) { // CALC_EXTERNAL
      typeName = "Binding";
      details = "Ref: <code>" + String(cfg.binding.tag) + "</code>";
    }

    // Publish status visualisering
    String pubStatus = cfg.publishValue ? "<span style='color:#4caf50;'>Yes</span>" : "<span style='color:#888;'>No</span>";

    html += "<td><p class='sensorHeaderSmallScreen'>Type</p>" + typeName + "</td>";
    html += "<td><p class='sensorHeaderSmallScreen'>Details</p>" + details + "</td>";
    html += "<td><p class='sensorHeaderSmallScreen'>Publish</p>" + pubStatus + "</td>";

    html += "<td class='actions'>";
    html += "<a class='btn-edit' href='/config/calc/edit?id=" + String(i) + "'>Edit</a>";
    html += "<button class='btn-delete' onclick='deleteCalculation(" + String(i) + ")'>Delete</button>";
    html += "</td></tr>";
  }
  html += "</tbody></table></div>";
  return html;
}


/* Summary of function: Generates HTML fields for both Add and Edit pages with dynamic validation fixes */
String getCalculationFormFields(const CalculationConfig* cfg = nullptr) {
  CalculationConfig defaultCfg;
  if (!cfg) {
    defaultCfg.type = 1; // CALC_EXPRESSION
    defaultCfg.publishValue = true;
    memset(defaultCfg.name, 0, sizeof(defaultCfg.name));
    memset(defaultCfg.unit, 0, sizeof(defaultCfg.unit));
    memset(defaultCfg.expression.expression, 0, sizeof(defaultCfg.expression.expression));
    defaultCfg.expression.triggerThreshold = 0.5f;
    defaultCfg.continuous.evalIntervalMs = 1000;
    defaultCfg.continuous.minValue = NAN;
    defaultCfg.continuous.maxValue = NAN;
    memset(defaultCfg.binding.tag, 0, sizeof(defaultCfg.binding.tag));
    defaultCfg.binding.fallbackValue = 0.0f;
    defaultCfg.binding.resetIntervalMs = 60000;
    cfg = &defaultCfg;
  }

  String html = "";

  // Type Selection
  html += "<div><label>Calculation Type</label>";
  html += "<select id='calcType' name='type' onchange='toggleCalcFields()' required>";
  html += "<option value='1' " + String(cfg->type == 1 ? "selected" : "") + ">Expression (Ternary/Trigger)</option>";
  html += "<option value='2' " + String(cfg->type == 2 ? "selected" : "") + ">Continuous (Min/Max/Interval)</option>";
  html += "<option value='3' " + String(cfg->type == 3 ? "selected" : "") + ">External (Remote/MQTT Binding)</option>";
  html += "</select></div>";

  // Metadata
  html += "<div><label>Name (Internal ID)</label><input type='text' name='name' value='" + String(cfg->name) + "' maxlength='15' required></div>";
  html += "<div><label>Unit</label><input type='text' name='unit' value='" + String(cfg->unit) + "' maxlength='7'></div>";

  html += "<label><input type='checkbox' id='publishValue' name='publishValue' value='1' " + String(cfg->publishValue ? "checked" : "") + "> Publish to Broker</label>";
  html += "<small style='display:block; margin-top: -1rem; color: #666;'>If disabled, the value remains internal.</small>";

  // Expression Input
  String currentExpr = (cfg->type == 2) ? String(cfg->continuous.expression) : String(cfg->expression.expression);
  html += "<div id='expressionInputWrapper'>";
  html += "<label>Expression</label>";
  html += "<input type='text' name='expression' id='expressionInput' value='" + currentExpr + "' maxlength='63'></div>";

  // External Tag Input
  html += "<div id='tagInputWrapper'>";
  html += "<label>Binding Tag (MQTT Topic or ID)</label>";
  html += "<input type='text' name='tag' value='" + String(cfg->type == 3 ? cfg->binding.tag : "") + "' maxlength='47' placeholder='e.g. sensors/temp/room1'></div>";

  // Contextual Fields (Type 1)
  html += "<div id='expressionFields'>";
  html += "  <div class='grid'><div><label>Trigger Threshold</label><input type='number' name='triggerThreshold' step='0.1' value='" + String(cfg->expression.triggerThreshold) + "'></div></div>";
  html += "</div>";

  // Contextual Fields (Type 2)
  html += "<div id='continuousFields'>";
  html += "  <div class='grid'>";
  html += "    <div><label>Interval (ms)</label><input type='number' name='interval' value='" + String(cfg->continuous.evalIntervalMs) + "' min='100'></div>";
  html += "    <div><label>Min Value</label><input type='number' name='min' step='0.1' value='" + (isnan(cfg->continuous.minValue) ? "" : String(cfg->continuous.minValue)) + "'></div>";
  html += "    <div><label>Max Value</label><input type='number' name='max' step='0.1' value='" + (isnan(cfg->continuous.maxValue) ? "" : String(cfg->continuous.maxValue)) + "'></div>";
  html += "  </div>";
  html += "</div>";

  // Contextual Fields (Type 3)
  html += "<div id='externalFields'>";
  html += "  <div class='grid'>";
  html += "    <div><label>Fallback Value</label><input type='number' name='fallbackValue' step='0.1' value='" + String(cfg->binding.fallbackValue) + "'></div>";
  html += "    <div><label>Reset Interval (ms)</label><input type='number' name='resetIntervalMs' value='" + String(cfg->binding.resetIntervalMs) + "'></div>";
  html += "  </div>";
  html += "</div>";

  // Updated Script with Section Disabling Logic
  html += "<script>";
  html += "function toggleCalcFields(){";
  html += "  const t = document.getElementById('calcType').value;";
  html += "  const toggle = (id, show) => {";
  html += "    const el = document.getElementById(id);";
  html += "    if(!el) return;";
  html += "    el.style.display = show ? 'block' : 'none';";
  // Disable all inputs within the hidden section so they don't trigger browser validation
  html += "    const inputs = el.querySelectorAll('input, select');";
  html += "    inputs.forEach(i => i.disabled = !show);";
  html += "  };";
  html += "  toggle('expressionFields', t == '1');";
  html += "  toggle('continuousFields', t == '2');";
  html += "  toggle('externalFields', t == '3');";
  html += "  toggle('expressionInputWrapper', t != '3');";
  html += "  toggle('tagInputWrapper', t == '3');";
  html += "}";
  html += "toggleCalcFields();";
  html += "</script>";

  return html;
}

/* Summary of function: Generates the HTML fragment for adding a new calculation slot including external bindings */
String displayAddCalculationFields() {
  String html = "<div><h2 id='addNewCalculation'>Add new calculation</h2>";

  int currentCount = expressionManager.getConfiguredCalculationCount();
  int availableSlots = MAX_CALCULATIONS_ALLOWED - currentCount;
  String slotColor = (availableSlots <= 0) ? "red" : "green";

  html += "<div class='slots-info'>Available slots: " + String(availableSlots);
  html += "Available slots: " + String(availableSlots) + " / " + String(MAX_CALCULATIONS_ALLOWED);
  html += "</div>";

  if (availableSlots <= 0) {
    html += "<div class='no-slots' style='color:var(--primary); font-weight:700;'>No available slots. Please delete to add a new one.</div>";
    html += "</div>";
    return html;
  }

  html += "<form id='calcForm' action='/config/calculation' method='POST' style='display:flex; flex-direction:column; gap:15px;'>";
  html += getCalculationFormFields(); // Helper handles everything
  html += "<button type='button' id='addCalcBtn' class='inheritSubmitStyle' style='margin-top:10px;'>Add Calculation</button>";
  html += "</form>";
  html += "</div>";

  html += "</div>";
  return html;
}

// Endpoint handler for the Edit Calculation page
/* Summary of function: Endpoint handler for the Edit Calculation page with AJAX save support */
void handleGetEditCalculationPage() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  if (!server.hasArg("id")) {
    asyncResponse({ "Missing calculation ID" }, false);
    return;
  }

  int calcId = server.arg("id").toInt();
  if (calcId < 0 || calcId >= MAX_CALCULATIONS_ALLOWED) {
    asyncResponse({ "Invalid ID" }, false);
    return;
  }

  const CalculationConfig& cfg = config.calculations[calcId];
  std::vector<String> htmlLines;

  htmlLines.push_back(getHTMLHeader("Edit Calculation"));
  htmlLines.push_back(mainBody("start"));
  htmlLines.push_back("<h1>Edit Calculation Slot " + String(calcId + 1) + "</h1>");

  // Form ID is important for the JavaScript selector
  htmlLines.push_back("<form id='editCalcForm' action='/config/calculation' method='POST' style='display:flex; flex-direction:column; gap:15px;'>");

  // Hidden ID field
  htmlLines.push_back("<input type='hidden' name='calcId' value='" + String(calcId) + "'>");

  // Dynamic Fields
  htmlLines.push_back(getCalculationFormFields(&cfg));

  // Action Buttons
  htmlLines.push_back("<div style='display:flex; gap:10px; margin-top:10px;'>");

  // Standard Submit (Legacy/Full Reload)
  htmlLines.push_back("<button type='submit' class='inheritSubmitStyle' style='flex:1;'>Save & Exit</button>");

  // New "Save and Stay" button using Fetch API
  htmlLines.push_back("<button type='button' onclick='SaveSettingsInline()' class='inheritSubmitStyle' style='flex:1; background-color:var(--secondary);'>Save Changes</button>");

  htmlLines.push_back("</div>");
  htmlLines.push_back("</form>");

  // Script for toggling and AJAX submission
  htmlLines.push_back("<script>");
  htmlLines.push_back("function toggleCalcFields(){ const t=document.getElementById('calcType').value; document.getElementById('expressionFields').style.display=(t=='1')?'block':'none'; document.getElementById('continuousFields').style.display=(t=='2')?'block':'none'; }");

  /* Summary of function: Submits the form data without reloading the page */
  htmlLines.push_back("async function SaveSettingsInline() {");
  htmlLines.push_back("  const form = document.getElementById('editCalcForm');");
  htmlLines.push_back("  const formData = new FormData(form);");
  htmlLines.push_back("  try {");
  htmlLines.push_back("    const response = await fetch(form.action, { method: 'POST', body: formData });");
  htmlLines.push_back("    if(response.ok) { alert('Changes saved successfully!'); }");
  htmlLines.push_back("    else { alert('Save failed.'); }");
  htmlLines.push_back("  } catch (err) { console.error(err); alert('Error connecting to server'); }");
  htmlLines.push_back("}");
  htmlLines.push_back("</script>");

  htmlLines.push_back(mainBody("end"));
  htmlLines.push_back(getHTMLFooter());

  asyncResponse(htmlLines);
}


/* Summary of function: JS handler for Actuator AJAX updates with explicit checkbox handling */
String actuatorTableScript() {
  String html = "<script id='updateActuatorTableScript'>";

  html += "async function updateActTable(url, options) {";
  html += "  try {";
  html += "    const response = await fetch(url, options);";
  html += "    if (!response.ok) throw new Error('Server error: ' + response.status);";
  html += "    const text = await response.text();";
  html += "    const parser = new DOMParser();";
  html += "    const doc = parser.parseFromString(text, 'text/html');";
  html += "    const newTable = doc.querySelector('#actuator-table');";
  html += "    const oldTable = document.getElementById('actuator-table');";
  html += "    if (oldTable && newTable) oldTable.replaceWith(newTable);";
  html += "  } catch (err) { alert('Failed to update: ' + err.message); }";
  html += "}";

  html += "document.addEventListener('click', async function(e) {";
  html += "  if (e.target && e.target.id === 'addActBtn') {";
  html += "    const form = document.getElementById('actuatorForm');";

  // Basic validation
  html += "    const name = form.querySelector('[name=name]').value.trim();";
  html += "    const source = form.querySelector('[name=sourceName]').value.trim();";
  html += "    if (!name || !source) { alert('Name and Mirror Source are required'); return; }";

  html += "    e.target.disabled = true;";
  html += "    const formData = new FormData(form);";

  html += "    formData.set('persistOnlyChange', form.querySelector('[name=persistOnlyChange]').checked ? '1' : '0');";
  // We check if the element exists because it might be hidden/disabled in Analog mode
  html += "    const invCb = form.querySelector('[name=inverted]');";
  html += "    if(invCb) formData.set('inverted', invCb.checked ? '1' : '0');";

  html += "    await updateActTable(form.action + '?responseFormat=html', { method: form.method, body: formData });";
  html += "    form.reset(); e.target.disabled = false;";
  html += "    if(typeof toggleActFields === 'function') toggleActFields();";
  html += "  }";
  html += "});";

  html += "function deleteActuator(id) {";
  html += "  if (confirm('Delete actuator slot ' + id + '?')) {";
  html += "    updateActTable('/config/actuator/delete?id=' + id + '&responseFormat=html', { method: 'GET' });";
  html += "  }";
  html += "}";

  html += "</script>";
  return html;
}

/* Summary of function: Displays configured actuator slots in a responsive table */
String displayActuatorsInProduction() {
  String html = "<div class='sensor-list' id='actuator-list'>";
  html += "<h2 id='acturatorHeader'>Configured Actuators</h2>";
  html += "<table class='sensor-table' id='actuator-table'>";
  html += "<thead><tr><th>ID</th><th>Name</th><th>Mirror Source</th><th>Hardware</th><th>Value</th><th>Actions</th></tr></thead>";
  html += "<tbody>";

  for (int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) {
    auto& act = config.actuators[i];
    if (act.type == ACTUATOR_UNCONFIGURED) continue;

    html += "<tr>";
    html += "<td><p class='sensorHeaderSmallScreen'>ID</p>" + String(i) + "</td>";
    html += "<td><p class='sensorHeaderSmallScreen'>Name</p><strong>" + String(act.name) + "</strong></td>";
    html += "<td><p class='sensorHeaderSmallScreen'>Source</p><code>" + String(act.sourceName) + "</code></td>";

    String hwDetails;
    if (act.type == ACTUATOR_DIGITAL) {
      hwDetails = "Digital (Pin " + String(act.digital.digitalPin) + (act.digital.inverted ? " INV" : "") + ")";
    } else {
      hwDetails = "Analog (Pin " + String(act.analog.analogPin) + ")";
    }

    html += "<td><p class='sensorHeaderSmallScreen'>Hardware</p>" + hwDetails + "</td>";
    html += "<td><p class='sensorHeaderSmallScreen'>Value</p>" + String(act.runtimeValue) + "</td>";

    html += "<td class='actions'>";
    html += "<a class='btn-edit' href='/config/actuator/edit?id=" + String(i) + "'>Edit</a>";
    html += "<button class='btn-delete' onclick='deleteActuator(" + String(i) + ")'>Delete</button>";
    html += "</td></tr>";
  }
  html += "</tbody></table></div>";
  return html;
}


/* Summary of function: Generates HTML fields for Actuator Add/Edit pages with consistent checkbox layouts and validation fixes */
String getActuatorFormFields(const ActuatorConfig* act = nullptr) {
  ActuatorConfig d;
  if (!act) {
    d.type = ACTUATOR_DIGITAL;
    memset(d.name, 0, sizeof(d.name));
    memset(d.sourceName, 0, sizeof(d.sourceName));
    d.digital.digitalPin = 0;
    d.digital.inverted = false;
    d.publishPolicy.trigger = PUBLISH_ON_THRESHOLD_OR_INTERVAL;  // was: persistOnlyChange = true
    act = &d;
  }

  String html = "";
  // Meta
  html += "<div><label>Actuator Name</label><input type='text' name='name' value='" + String(act->name) + "' maxlength='15' required></div>";
  html += "<div><label>Mirror Source Name</label><input type='text' name='sourceName' value='" + String(act->sourceName) + "' maxlength='31' required placeholder='e.g. MyLogic_Result'></div>";

  // Type
  html += "<div><label>Output Type</label>";
  html += "<select id='actType' name='type' onchange='toggleActFields()' required>";
  html += "<option value='0' " + String(act->type == 0 ? "selected" : "") + ">Digital (High/Low)</option>";
  html += "<option value='1' " + String(act->type == 1 ? "selected" : "") + ">Analog (PWM/DAC)</option>";
  html += "</select></div>";

  // Digital
  html += "<div id='digitalActFields'>";
  html += "  <div class='grid'><div><label>GPIO Pin</label><input type='number' name='digitalPin' value='" + String(act->type == 0 ? act->digital.digitalPin : 0) + "'></div></div>";
  html += "  <label><input type='checkbox' name='inverted' value='1' " + String((act->type == 0 && act->digital.inverted) ? "checked" : "") + "> Inverted Logic</label>";
  html += "  <small style='display:block; margin-top: -1rem; color: #666;'>Active-Low: ON results in 0V (LOW), OFF results in 3.3V (HIGH).</small>";
  html += "</div>";

  // Analog Section
  html += "<div id='analogActFields'>";
  html += "  <div class='grid'>";
  html += "    <div><label>GPIO</label><input type='number' name='analogPin' value='" + String(act->type == 1 ? act->analog.analogPin : 0) + "'></div>";
  html += "    <div><label>Logic Min (%)</label><input type='number' step='0.1' name='minValue' value='" + String(act->type == 1 ? act->analog.minValue : 0) + "'></div>";
  html += "  </div>";
  html += "  <div class='grid'>";
  html += "    <div><label>Logic Max (%)</label><input type='number' step='0.1' name='maxValue' value='" + String(act->type == 1 ? act->analog.maxValue : 100) + "'></div>";
  html += "    <div><label>Unit</label><input type='text' name='unit' value='" + String(act->type == 1 ? act->analog.unit : "%") + "'></div>";
  html += "  </div>";

  // New Voltage Calibration Fields
  html += "  <div class='grid'>";
  html += "    <div><label>Hardware Min (V)</label><input type='number' step='0.01' name='voltageMin' value='" + String(act->type == 1 ? act->analog.voltageMin : 0.0) + "'></div>";
  html += "    <div><label>Hardware Max (V)</label><input type='number' step='0.01' name='voltageMax' value='" + String(act->type == 1 ? act->analog.voltageMax : 3.3) + "'></div>";
  html += "  </div>";
  html += "  <small style='display:block; color: #666;'>Hardware Voltage: Map 0-100% logic to a specific voltage swing (Native: 0V - 3.3V).</small>";
  html += "</div>";

  // Global Actuator Checkboxes
  // Global Actuator Publish Policy
  html += "<div><label>Publish Mode</label>";
  html += "<select name='publishTrigger'>";
  html += "<option value='0' " + String(act->publishPolicy.trigger == PUBLISH_ON_THRESHOLD_OR_INTERVAL ? "selected" : "") + ">Threshold or Interval</option>";
  html += "<option value='1' " + String(act->publishPolicy.trigger == PUBLISH_ON_CHANGE_OF_THRESHOLD   ? "selected" : "") + ">Changes only (threshold)</option>";
  html += "<option value='2' " + String(act->publishPolicy.trigger == PUBLISH_ON_INTERVAL              ? "selected" : "") + ">Interval only</option>";
  html += "</select></div>";
  html += "<small style='display:block; color: #666;'>Controls when the actuator's applied value is reported back to the broker.</small>";

  html += "<script>";
  html += "function toggleActFields(){";
  html += "  const t = document.getElementById('actType').value;";
  html += "  const toggle = (id, show) => {";
  html += "    const el = document.getElementById(id);";
  html += "    if(!el) return;";
  html += "    el.style.display = show ? 'block' : 'none';";
  // Disable hidden inputs to avoid "not focusable" validation errors
  html += "    const inputs = el.querySelectorAll('input, select');";
  html += "    inputs.forEach(i => i.disabled = !show);";
  html += "  };";
  html += "  toggle('digitalActFields', t == '0');";
  html += "  toggle('analogActFields', t == '1');";
  html += "}";
  html += "toggleActFields();";
  html += "</script>";

  return html;
}


/* Summary of function: Generates the HTML section for adding a new actuator with English labels and English comments */
String DisplayAddActuatorFields() {
  String html = "<div class='actuator-form-section' style='margin-top:2vh;'>";
  html += "<h2 id='addNewActuator'>Add New Actuator</h2>";

  int currentCount = actuatorManager.getConfiguredActuatorCount();
  int availableSlots = MAX_ACTUATORS_ALLOWED - currentCount;
  String slotColor = (availableSlots <= 0) ? "red" : "green";

  // Display slot availability status
  html += "<div class='slots-info' style='margin-bottom:1vh; color:" + slotColor + ";'>";
  html += "Available slots: " + String(availableSlots) + " / " + String(MAX_ACTUATORS_ALLOWED);
  html += "</div>";

  if (availableSlots <= 0) {
    html += "<p style='color:var(--primary); font-weight:700;'>No available slots. Please delete an existing actuator first.</p>";
    html += "</div>";
    return html;
  }

  // Define the form with the specific ID for JS interception
  html += "<form id='actuatorForm' action='/config/actuator' method='POST' style='display:flex; flex-direction:column; gap:15px;'>";

  // Inject the reusable input fields
  html += getActuatorFormFields();

  // The specific button ID 'addActBtn' is required for the AJAX script
  html += "<button type='submit' class='inheritSubmitStyle' style='margin-top:10px;'>Add Actuator</button>";

  html += "</form>";
  html += "</div>";

  return html;
}


void handleGetEditActuatorPage() {
  if (!isAuthenticated()) { server.requestAuthentication(); return; }
  if (!server.hasArg("id")) { asyncResponse({"Missing ID"}, false); return; }

  int id = server.arg("id").toInt();
  if (id < 0 || id >= MAX_ACTUATORS_ALLOWED) { asyncResponse({"Invalid ID"}, false); return; }

  const ActuatorConfig& act = config.actuators[id];
  std::vector<String> html;
  html.push_back(getHTMLHeader("Edit Actuator"));
  html.push_back(mainBody("start"));
  html.push_back("<h1>Edit Actuator Slot " + String(id) + "</h1>");
  html.push_back("<form action='/config/actuator' method='POST' style='display:flex; flex-direction:column; gap:15px;'>");
  html.push_back("<input type='hidden' name='actId' value='" + String(id) + "'>");
  html.push_back(getActuatorFormFields(&act));
  html.push_back("<button type='submit' class='inheritSubmitStyle'>Save Changes</button>");
  html.push_back("</form>");
  html.push_back(mainBody("end"));
  html.push_back(getHTMLFooter());
  asyncResponse(html);
}





void handleCertificatePages() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  String html = getHTMLHeader("Certificate Management");
  html += mainBody("start");
  html += "<h1>Device Certificate Management</h1>";

  html += getSimpleDeviceInfoHTML();

  // Display currently active certificate summary
  html += "<h2 id='activeCertificate'>Active Certificate</h2>";
  html += "<textarea readonly rows='10' style='width:100%; min-width:100%; resize:none; font-family:monospace;'>";
  //html += certificateManager.getCertificateSummary();  // Returns info like issuer, subject, valid dates
  html += "</textarea><br><br>";

  // Validate certificate button
  html += "<h2>Validate Certificate</h2>";
  html += "<form method='POST' action='/certificate/validate'>";
  html += "<input type='submit' value='Validate Certificate'>";
  html += "</form><br>";

  // Upload new certificate + private key
  html += "<h2>Upload New Certificate</h2>";
  html += "<form method='POST' action='/certificate/upload' enctype='multipart/form-data'>";
  html += "Certificate (.pem):<br>";
  html += "<input type='file' name='certfile' accept='.pem' required><br><br>";
  html += "Private Key (.pem):<br>";
  html += "<input type='file' name='keyfile' accept='.pem' required><br><br>";
  html += "<input type='submit' value='Upload Certificate'>";
  html += "</form><br>";

  // Delete current certificate
  html += "<h2>Delete Certificate</h2>";
  html += "<form method='POST' action='/certificate/delete'>";
  html += "<input type='submit' value='Delete Certificate'>";
  html += "</form><br>";

  html += mainBody("end");

  html += getHTMLFooter();
  server.send(200, "text/html", html);
}









void handleRoot() {
  if (DEBUG) Serial.println("Kall for ROOT...");

  if (!isAuthenticated()) return server.requestAuthentication();

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  // --- Header ---
  server.sendContent(getHTMLHeader("Configuration"));
  // Pause wifi packages
  yield();

  server.sendContent(mainBody("start"));
  server.sendContent("<h1>Grefur Backbone Configuration</h1>");

  // --- Content sections ---
  server.sendContent(getDeviceInfoHTML());
  server.sendContent(getLedControlHTML());
  server.sendContent(getDeviceInformationForm());
  server.sendContent(getWiFiConfigForm());
  server.sendContent(DisplayMQTTSettingsHTML());
  server.sendContent(DisplayOperationSettings());

  // Pause wifi packages
  yield();

  server.sendContent(DisplaySensorsInProduction());
  server.sendContent(DisplayAddSensorFields());

  /* Render Actuator Section */
  server.sendContent(displayActuatorsInProduction()); // The table of existing actuators
  server.sendContent(DisplayAddActuatorFields());      // The form to add new ones

  server.sendContent(calculationTableScript());
  server.sendContent(displayAddCalculationFields());
  server.sendContent(displayCalculationsInProduction());

  server.sendContent(mainBody("end"));

  // --- Scripts at bottom ---
  server.sendContent(SensorTableScript());     // JS for form + delete
  server.sendContent(actuatorTableScript());         // The JS logic for the table and buttons
  server.sendContent(getAjaxFormScript());     // JS for statustext below each form
  server.sendContent(SensorTypeFieldsScript()); // JS for dynamic sensor registration fields

  // --- Footer ---
  server.sendContent(getHTMLFooter());

  // Empty string for end marker
  server.sendContent("");
  if (server.client()) {
    server.client().flush(); // Delay for buffer to be empty
    delay(1);               // Small delay
  }

}



// GET request for uploading new firmware
void firmwareVersionPage() {
  if (!isAuthenticated()) return server.requestAuthentication();

  String html = getHTMLHeader("Firmware Update");
  html += mainBody("start");
  html += "<h1>Firmware Update</h1>";
  html += "<div class='info'>";
  html += getDeviceIdParagraph();
  html += "<p><strong>Firmware Version:</strong> " + getFirmwareVersion() + "</p>";
  html += "<p><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</p>";
  html += "</div>";
  html += "<h2>Upload New Firmware</h2>";
  html += "<form method='POST' action='/firmware' enctype='multipart/form-data'>";
  html += "<input type='file' name='firmware'><br><br>";
  html += "<input type='submit' value='Upload & Update'>";
  html += "</form>";
  html += mainBody("end");
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}



void backupPage() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  String html = getHTMLHeader("Backup");
  html += mainBody("start");
  html += "<h1>Configuration Backup</h1>";

  html += getSimpleDeviceInfoHTML();

  // Backup download
  html += "<h2>Create Backup</h2>";
  html += "<button onclick=\"window.location.href='/backup/file'\">Download Backup</button>";

  // Restore from JSON upload
  html += "<h2>Restore Backup</h2>";
  html += "<form method='POST' action='/backup/upload' enctype='multipart/form-data'>";
  html += "<input type='file' name='backupfile' accept='.json' required><br><br>";
  html += "<input type='submit' value='Upload and Restore'>";
  html += "</form>";
  html += mainBody("end");
  html += getHTMLFooter();
  server.send(200, "text/html", html);
}








void handleGetEditPage() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  if (!server.hasArg("id")) {
    asyncResponse({ "Missing sensor ID" }, false);
    return;
  }

  int sensorId = server.arg("id").toInt();
  if (sensorId < 0 || sensorId >= MAX_SENSORS_ALLOWED) {
    asyncResponse({ "Invalid sensor ID" }, false);
    return;
  }

  const SensorConfig& sensor = sensorManager.getSensorConfig(sensorId);
  String lastMeasurement = String(sensorManager.getLastMeasurement(sensorId));
  String unit = sensorManager.getSensorUnit(sensorId);

  std::vector<String> htmlLines;
  htmlLines.push_back(getHTMLHeader("Sensor Edit"));
  htmlLines.push_back(mainBody("start"));
  htmlLines.push_back("<h1>Sensor Edit Page " + String(sensorId+1) + "</h1>");
  htmlLines.push_back(getSimpleDeviceInfoHTML(
                        (lastMeasurement != "nan" && lastMeasurement != "") ? lastMeasurement + " " + unit : ""
                      ));

  htmlLines.push_back("<form id='sensorForm' action='/config/sensor' method='POST'>");
  htmlLines.push_back("<input type='hidden' name='sensorId' value='" + String(sensorId) + "'>");
  htmlLines.push_back("<input type='hidden' name='name' value='" + String(sensor.name) + "'>");
  htmlLines.push_back("<input type='hidden' name='type' value='" + String(sensor.type) + "'>");
  htmlLines.push_back("<div id='dynamicFields'></div>");

  htmlLines.push_back("<div style='display:flex; gap:10px; margin-top:10px;'>");
  htmlLines.push_back("<button type='submit' class='inheritSubmitStyle' style='flex:1;'>Save & Exit</button>");
  htmlLines.push_back("<button type='button' onclick='SaveSettingsInline()' class='inheritSubmitStyle' style='flex:1; background-color:var(--secondary);'>Save Changes</button>");
  htmlLines.push_back("</div>");
  htmlLines.push_back("</form>");

  htmlLines.push_back("<script>");
  htmlLines.push_back("const container = document.getElementById('dynamicFields');");
  htmlLines.push_back(SensorTypeFieldsScript(sensor.type));
  htmlLines.push_back("setTimeout(() => {");

  if (sensor.type == SENSOR_NTC) {
    htmlLines.push_back("container.querySelector('[name=seriesResistor]').value = '" + String(sensor.ntc.seriesResistor) + "';");
    htmlLines.push_back("container.querySelector('[name=nominalResistance]').value = '" + String(sensor.ntc.nominalResistance) + "';");
    htmlLines.push_back("container.querySelector('[name=nominalTemp]').value = '" + String(sensor.ntc.nominalTemp) + "';");
    htmlLines.push_back("container.querySelector('[name=bCoefficient]').value = '" + String(sensor.ntc.bCoefficient) + "';");
    htmlLines.push_back("container.querySelector('[name=unit]').value = '" + String(sensor.ntc.unit) + "';");
    htmlLines.push_back("container.querySelector('[name=analogPin]').value = '" + String(sensor.ntc.analogPin) + "';");
    htmlLines.push_back("container.querySelector('[name=publishTrigger]').value = '" + String(sensor.publishPolicy.trigger) + "';");
    htmlLines.push_back("container.querySelector('[name=changeThreshold]').value = '" + String(sensor.publishPolicy.changeThreshold) + "';");
    htmlLines.push_back("container.querySelector('[name=intervalMs]').value = '" + String(sensor.publishPolicy.intervalMs) + "';");
  }
  else if (sensor.type == SENSOR_ANALOG) {
    htmlLines.push_back("container.querySelector('[name=analogPin]').value = '" + String(sensor.analog.analogPin) + "';");
    htmlLines.push_back("container.querySelector('[name=inputMin]').value = '" + String(sensor.analog.inputMin) + "';");
    htmlLines.push_back("container.querySelector('[name=inputMax]').value = '" + String(sensor.analog.inputMax) + "';");
    htmlLines.push_back("container.querySelector('[name=scaleMin]').value = '" + String(sensor.analog.scaleMin) + "';");
    htmlLines.push_back("container.querySelector('[name=scaleMax]').value = '" + String(sensor.analog.scaleMax) + "';");
    htmlLines.push_back("container.querySelector('[name=scaleUnit]').value = '" + String(sensor.analog.scaleUnit) + "';");
    htmlLines.push_back("container.querySelector('[name=publishTrigger]').value = '" + String(sensor.publishPolicy.trigger) + "';");
    htmlLines.push_back("container.querySelector('[name=changeThreshold]').value = '" + String(sensor.publishPolicy.changeThreshold) + "';");
    htmlLines.push_back("container.querySelector('[name=intervalMs]').value = '" + String(sensor.publishPolicy.intervalMs) + "';");
  }
  else if (sensor.type == SENSOR_DIGITAL) {
    htmlLines.push_back("container.querySelector('[name=digitalPin]').value = '" + String(sensor.digital.digitalPin) + "';");
    htmlLines.push_back("container.querySelector('[name=inverted]').checked = " + String(sensor.digital.inverted ? "true" : "false") + ";");
    htmlLines.push_back("container.querySelector('[name=publishTrigger]').value = '" + String(sensor.publishPolicy.trigger) + "';");
    htmlLines.push_back("container.querySelector('[name=intervalMs]').value = '" + String(sensor.publishPolicy.intervalMs) + "';");
  }

  htmlLines.push_back("}, 50);");

  htmlLines.push_back("async function SaveSettingsInline() {");
  htmlLines.push_back("  const form = document.getElementById('sensorForm');");
  htmlLines.push_back("  const formData = new FormData(form);");
  htmlLines.push_back("  try {");
  htmlLines.push_back("    const response = await fetch(form.action, { method: 'POST', body: formData });");
  htmlLines.push_back("    if(response.ok) { alert('Changes saved successfully!'); }");
  htmlLines.push_back("    else { alert('Save failed.'); }");
  htmlLines.push_back("  } catch (err) { console.error(err); alert('Error connecting to server'); }");
  htmlLines.push_back("}");
  htmlLines.push_back("</script>");

  htmlLines.push_back(mainBody("end"));
  htmlLines.push_back(getHTMLFooter());

  asyncResponse(htmlLines);
}









/*
   @brief ROUTEHANDLERS WITH POST AND GET


*/

void handleWifiSettingsChange() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  // Capture original values
  char originalSSID[sizeof(config.network.wifiSSID)];
  char originalPass[sizeof(config.network.wifiPassword)];
  strncpy(originalSSID, config.network.wifiSSID, sizeof(originalSSID));
  strncpy(originalPass, config.network.wifiPassword, sizeof(originalPass));

  // Update fields
  bool ssidChanged = mgr.updateStringField(config.network.wifiSSID,
                     sizeof(config.network.wifiSSID),
                     server.arg("ssid"));
  bool passChanged = mgr.updateStringField(config.network.wifiPassword,
                     sizeof(config.network.wifiPassword),
                     server.arg("password"));

  mgr.setSendHtml(true);

  // Handle changes
  if (mgr.hasChanges()) {
    mgr.scheduleRestart();
    mgr.finalize("WiFi settings updated");
  } else {
    mgr.finalize("No changes detected");
  }
}


void handleEthernetSettingsChange() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  // Capture original values
  bool originalUseDHCP = config.network.useDHCP;
  IPAddress originalStaticIP = config.network.staticIP;
  IPAddress originalGateway = config.network.gateway;
  IPAddress originalSubnet = config.network.subnet;

  // Update fields from POST request
  config.network.useDHCP = server.hasArg("useDHCP"); // Checkbox returns true if present

  // Only update static fields if DHCP is false
  if (!config.network.useDHCP) {
    mgr.updateIPAddressField(config.network.staticIP, server.arg("staticIP"));
    mgr.updateIPAddressField(config.network.gateway, server.arg("gateway"));
    mgr.updateIPAddressField(config.network.subnet, server.arg("subnet"));
  }

  // Handle changes
  if (mgr.hasChanges()) {
    mgr.scheduleRestart();
    mgr.setSendHtml(true);
    mgr.finalize("Ethernet settings updated");
  }
}




/**
   @brief Handle MQTT configuration updates

   Processes and saves MQTT broker settings including:
   - Connection parameters (URL, port)
   - Credentials (username/password)
   - Publishing options (QoS, retain flag)
   Schedules restart if changes were made.
*/
/* Summary of function: Handles MQTT configuration updates from the web interface.
   Manages the transition between Grefur-managed and manual connection modes. */
void handleMQTTSettingsChange() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  // 1. Check if Grefur Services is requested
  bool grefurRequested = server.hasArg("useGrefur");
  String mqttTopic = server.arg("mqttTopic");

  // Grefur Topology Guard: If Grefur is requested and topic is blank, use default
  if (grefurRequested && mqttTopic.length() == 0) {
    mqttTopic = BASETOPIC_DEFAULT;
  }

  // If user is enabling Grefur Services now
  if (grefurRequested && !config.mqtt.useGrefurBackend) {
    mgr.updateBoolField(config.mqtt.useGrefurBackend, true);

    // Force-enable required operation settings for Grefur-Sensor ecosystem
    mgr.updateBoolField(config.operation.publishStatus, true);
    mgr.updateBoolField(config.operation.publishDeviceInfo, true);

    // Clear credentials to force a fresh authenticate() call on next boot
    memset(config.mqtt.username, 0, sizeof(config.mqtt.username));
    memset(config.mqtt.password, 0, sizeof(config.mqtt.password));

    #if DEBUG_MQTT_MANAGER
    Serial.println("[WEB] Grefur Services enabled. Credentials cleared for re-auth.");
    #endif
  }

  // 2. Process string fields (Note: mqttTopic is now pre-validated)
  mgr.updateStringField(config.mqtt.brokerURL, sizeof(config.mqtt.brokerURL), server.arg("mqttServer"));
  mgr.updateStringField(config.mqtt.clientID, sizeof(config.mqtt.clientID), server.arg("mqttClientId"));
  mgr.updateStringField(config.mqtt.username, sizeof(config.mqtt.username), server.arg("mqttUser"));
  mgr.updateStringField(config.mqtt.password, sizeof(config.mqtt.password), server.arg("mqttPassword"));
  mgr.updateStringField(config.mqtt.baseTopic, sizeof(config.mqtt.baseTopic), mqttTopic);

  // 3. Logic to disable Grefur Services if manual changes are made without the checkbox
  if (!grefurRequested && mgr.hasChanges()) {
    mgr.updateBoolField(config.mqtt.useGrefurBackend, false);
  }

  // 4. Update numeric and boolean settings
  mgr.updateUint16Field(config.mqtt.port, server.arg("mqttPort"));
  mgr.updateNumericField(config.mqtt.qos, server.arg("mqttQos"));

  mgr.updateBoolField(config.mqtt.retain, mgr.stringToBoolValue(server.arg("mqttRetain")));
  mgr.updateBoolField(config.mqtt.useNestedJson, mgr.stringToBoolValue(server.arg("useNestedJson")));

  // 5. Finalize: Save and restart if changes occurred
  if (mgr.hasChanges()) {
    mgr.scheduleRestart();
  }

  mgr.setSendHtml(true);
  mgr.finalize("MQTT settings updated");
}

/* Summary of function: Handles web request to update operation settings.
   Ensures publishInterval is at least 100ms for system stability. */
void handleOperationsSettingsChange() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  mgr.updateNumericField(config.operation.publishInterval, server.arg("publishInterval"));

  // Sjekk og korriger intervall hvis det er for lavt
  if (config.operation.publishInterval < 100) {
    config.operation.publishInterval = 100;
  }

  mgr.updateStringField(config.device.adminPass, sizeof(config.device.adminPass), server.arg("configPassword"));

  mgr.updateBoolField(config.operation.publishMeasuredValues, mgr.stringToBoolValue(server.arg("publishTemperature")));
  mgr.updateBoolField(config.operation.publishIP, mgr.stringToBoolValue(server.arg("publishIP")));
  mgr.updateBoolField(config.operation.publishStatus, mgr.stringToBoolValue(server.arg("publishStatus")));
  mgr.updateBoolField(config.operation.publishDeviceInfo, mgr.stringToBoolValue(server.arg("publishDeviceInfo")));
  mgr.updateBoolField(config.operation.publishRSSI, mgr.stringToBoolValue(server.arg("publishRSSI")));

  if (mgr.hasChanges()) {
    mgr.scheduleRestart();
  }
  mgr.setSendHtml(true);
  mgr.finalize("Operation settings updated");
}


/**
 * @brief Handles POST requests to /config/device to update device name and location.
 * * Uses SaveConfigManager for authentication, validation, and storage of changes,
 * and schedules a restart if the configuration was modified.
 */
void handleDeviceInfo() {
  // 1. Initialize the configuration manager
  SaveConfigManager mgr(config);

  // 2. Authenticate that the request is valid (e.g., by checking the admin password)
  if (!mgr.authenticateRequest()) return;

  // 3. Update the deviceName field
  mgr.updateStringField(
    config.device.deviceName,
    sizeof(config.device.deviceName), // Must be the defined size of the char array
    server.arg("deviceName")
  );

  // 4. Update the location field
  mgr.updateStringField(
    config.device.location,
    sizeof(config.device.location), // Must be the defined size of the char array
    server.arg("deviceLocation")
  );

  // 5. Check if any fields were changed
  if (mgr.hasChanges()) {
    // Schedule the device for restart (usually necessary to apply new network settings or device identity changes)
    mgr.scheduleRestart();
  }

  // 6. Finalize the request with feedback to the user
  mgr.setSendHtml(true);
  mgr.finalize("Device information successfully updated");
}







/*
  Flexible Handling:
    Returns fields for a specific sensor type if type parameter provided
    Returns all available types if no parameter specified
*/
void handleSensorTypeRequest() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) return;

  // Handle GET parameter for getting sensortypes
  if (server.method() == HTTP_GET) {
    if (server.hasArg("type")) {
      // Hent og valider type-parameter
      String typeStr = server.arg("type");
      typeStr.toLowerCase();

      int8_t type = -1;
      if (typeStr == "ntc") {
        type = SENSOR_NTC;
      } else if (typeStr == "analog") {
        type = SENSOR_ANALOG;
      } else if (typeStr == "digital") {
        type = SENSOR_DIGITAL;
      }

      if (type != -1) {
        String response = sensorManager.retrieveSensorFieldsAsJson(type);
        server.send(200, "application/json", response);
      } else {
        server.send(400, "text/plain", "Not compatible sensortype'");
      }
    } else {
      // Hvis ingen type er spesifisert, returner alle typer
      String response = sensorManager.retrieveSensorTypes();
      server.send(200, "application/json", response);
    }
  } else {
    server.send(405, "text/plain", "Method not supported");
  }
}


void handleSensorConfig() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  // 1. Identify if this is an Edit or New addition first
  String sensorIdStr = server.hasArg("sensorId") ? server.arg("sensorId") : "";
  int existingId = (sensorIdStr != "") ? sensorIdStr.toInt() : -1;
  bool isEdit = (sensorIdStr != "" && existingId >= 0 && existingId < MAX_SENSORS_ALLOWED);

  // Check available slots first
  if (!isEdit && sensorManager.getConfiguredSensorCount() >= MAX_SENSORS_ALLOWED) {
    server.send(409, "text/plain", "Maximum number of sensors reached (" + String(MAX_SENSORS_ALLOWED) + ")");
    return;
  }

  SensorConfig newConfig;
  memset(&newConfig, 0, sizeof(SensorConfig)); // Clear memory

  // Read and validate common fields
  String typeStr = server.arg("type");
  String name = server.arg("name");

  if (name.length() >= sizeof(newConfig.name)) {
    server.send(400, "text/plain", "Sensor name too long");
    return;
  }

  Serial.println("=== Sensor Config Submission ===");
  Serial.println("Type: " + typeStr);
  Serial.println("Name: " + name);

  strncpy(newConfig.name, name.c_str(), sizeof(newConfig.name));
  newConfig.name[sizeof(newConfig.name) - 1] = '\0'; // Ensure null-terminated

  if (typeStr == "ntc" || typeStr == "0") {
    newConfig.type = SENSOR_NTC;
    newConfig.ntc.seriesResistor = server.arg("seriesResistor").toFloat();
    newConfig.ntc.nominalResistance = server.arg("nominalResistance").toFloat();
    newConfig.ntc.nominalTemp = server.arg("nominalTemp").toFloat();
    newConfig.ntc.bCoefficient = server.arg("bCoefficient").toFloat();
    newConfig.ntc.analogPin = server.arg("analogPin").toInt();

    uint8_t triggerVal = (uint8_t)server.arg("publishTrigger").toInt();
    newConfig.publishPolicy.trigger = (triggerVal <= PUBLISH_ON_INTERVAL)
                                      ? (PublishTrigger)triggerVal
                                      : PUBLISH_ON_THRESHOLD_OR_INTERVAL;
    newConfig.publishPolicy.changeThreshold = server.arg("changeThreshold").toFloat();
    Serial.println("PublishTrigger: " + String(newConfig.publishPolicy.trigger));

    newConfig.publishPolicy.intervalMs = server.arg("intervalMs").toInt();

    String unitStr = server.arg("unit");
    strncpy(newConfig.ntc.unit, unitStr.c_str(), sizeof(newConfig.ntc.unit) - 1);
    newConfig.ntc.unit[sizeof(newConfig.ntc.unit) - 1] = '\0';

    Serial.println("NTC Config:");
    Serial.println("  seriesResistor: " + String(newConfig.ntc.seriesResistor));
    Serial.println("  nominalResistance: " + String(newConfig.ntc.nominalResistance));
    Serial.println("  nominalTemp: " + String(newConfig.ntc.nominalTemp));
    Serial.println("  bCoefficient: " + String(newConfig.ntc.bCoefficient));
    Serial.println("  analogPin: " + String(newConfig.ntc.analogPin));
    Serial.println("  unit: " + String(newConfig.ntc.unit));
  }
  else if (typeStr == "analog" || typeStr == "1") {
    newConfig.type = SENSOR_ANALOG;
    newConfig.analog.analogPin = server.arg("analogPin").toInt();
    newConfig.analog.inputMin = server.arg("inputMin").toFloat();
    newConfig.analog.inputMax = server.arg("inputMax").toFloat();
    newConfig.analog.scaleMin = server.arg("scaleMin").toFloat();
    newConfig.analog.scaleMax = server.arg("scaleMax").toFloat();

    // For digital, threshold is meaningless — clamp to interval-compatible modes only
    uint8_t triggerVal = (uint8_t)server.arg("publishTrigger").toInt();
    newConfig.publishPolicy.trigger = (triggerVal == PUBLISH_ON_INTERVAL)
                                      ? PUBLISH_ON_INTERVAL
                                      : PUBLISH_ON_THRESHOLD_OR_INTERVAL;

    newConfig.publishPolicy.changeThreshold = server.arg("changeThreshold").toFloat();
    Serial.println("PublishTrigger: " + String(newConfig.publishPolicy.trigger));
    Serial.println("ChangeThreshold: " + String(newConfig.publishPolicy.changeThreshold));

    newConfig.publishPolicy.intervalMs = server.arg("intervalMs").toInt();

    String scaleUnitStr = server.arg("scaleUnit");
    strncpy(newConfig.analog.scaleUnit, scaleUnitStr.c_str(), sizeof(newConfig.analog.scaleUnit) - 1);
    newConfig.analog.scaleUnit[sizeof(newConfig.analog.scaleUnit) - 1] = '\0';

    Serial.println("Analog Config:");
    Serial.println("  analogPin: " + String(newConfig.analog.analogPin));
    Serial.println("  inputMin: " + String(newConfig.analog.inputMin));
    Serial.println("  inputMax: " + String(newConfig.analog.inputMax));
    Serial.println("  scaleMin: " + String(newConfig.analog.scaleMin));
    Serial.println("  scaleMax: " + String(newConfig.analog.scaleMax));
    Serial.println("  scaleUnit: " + String(newConfig.analog.scaleUnit));
  }
  else if (typeStr == "digital" || typeStr == "2") {
    newConfig.type = SENSOR_DIGITAL;
    newConfig.digital.digitalPin = server.arg("digitalPin").toInt();
    newConfig.digital.inverted = server.arg("inverted") == "on";

    // For digital, threshold is meaningless — clamp to interval-compatible modes only
    uint8_t triggerVal = (uint8_t)server.arg("publishTrigger").toInt();
    newConfig.publishPolicy.trigger = (triggerVal == PUBLISH_ON_INTERVAL)
                                      ? PUBLISH_ON_INTERVAL
                                      : PUBLISH_ON_THRESHOLD_OR_INTERVAL;
    newConfig.publishPolicy.intervalMs = server.arg("intervalMs").toInt();  // <-- legg til

    Serial.println("PublishTrigger: " + String(newConfig.publishPolicy.trigger));
    Serial.println("Digital Config:");
    Serial.println("  digitalPin: " + String(newConfig.digital.digitalPin));
    Serial.println("  inverted: " + String(newConfig.digital.inverted ? "true" : "false"));
  }
  else {
    Serial.println("Unknown sensor type: " + typeStr);
    server.send(400, "text/plain", "Invalid sensor type");
    return;
  }


  // Add sensor config
  Serial.println("Attempting to add sensor config...");

  int sensorId = -1;

  if (isEdit) {
    // Overwrite existing slot - count remains same
    sensorId = sensorManager.updateSensor(existingId, newConfig);
  } else {
    // Create new slot - count increases
    sensorId = sensorManager.addSensor(newConfig);
  }

  sensorManager.saveChanges();

  if (sensorId >= 0) {
    if (isEdit) {
      String redirectComponent = "#configuredSensors";
      server.sendHeader("Location", String("/") + redirectComponent);
      server.send(303);
    } else {
      server.send(200, "text/html", DisplaySensorsInProduction());
    }
  } else {
    server.send(500, "text/plain", "Failed to save sensor");
  }
}

void handleDeleteSensor() {
  SaveConfigManager mgr(config);

  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Missing sensor ID");
    return;
  }

  String responseFormat = server.arg("responseFormat");
  // Blank response request
  if (responseFormat != "html" ){
    responseFormat = "json";
  }

  int id = server.arg("id").toInt();
  sensorManager.printSensorStates();

  if (sensorManager.removeSensor(id)) {
    Serial.println("Sensor marked for deletion in RAM");

    if (sensorManager.saveChanges()) {
      // Verify deletion in RAM (no EEPROM read needed)
      if (config.sensors[id].type == SENSOR_UNCONFIGURED) {
        if (responseFormat){
          String updatedTable = DisplaySensorsInProduction();
          server.send(200, "text/html", updatedTable);
        } else {
          server.send(200, "application/json",
                      "{\"status\":\"success\",\"id\":" + String(id) + "}");
        }

      } else {
        server.send(500, "application/json",
                    "{\"error\":\"RAM state inconsistent\"}");
      }
    } else {
      server.send(500, "application/json",
                  "{\"error\":\"EEPROM save failed\"}");
    }
  } else {
    server.send(400, "application/json",
                "{\"error\":\"Deletion failed\"}");
  }
}


void handleGetAllSensors() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }


  // Set CORS headers if needed
  server.sendHeader("Access-Control-Allow-Origin", "*");

  // Get the sensor data
  bool returnEverySensor = true;
  String sensorData = sensorManager.retrieveAllSensors(returnEverySensor);

  // Check if we got valid data
  if (sensorData.length() == 0 || sensorData == "{\"sensors\":[]}") {
    server.send(200, "application/json", "{\"status\":\"no sensors configured\"}");
    return;
  }

  // Send the response
  server.send(200, "application/json", sensorData);

  // Debug output
  if (DEBUG) {
    Serial.println("Sent sensor data:");
    Serial.println(sensorData);
  }

}

// POST request new firmware and reboot
/* Summary of function: Processes incoming firmware file chunks via multipart upload.
   Manages flash memory writing and ensures data integrity. */
void firmwareUploadPage() {
  // Access the current upload status
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("[OTA] Updating: %s\n", upload.filename.c_str());

    // Calculate available sketch space with safe offset
    uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

    // Start the update process
    if (!Update.begin(maxSketchSpace)) {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_WRITE) {
    // Write the current buffer chunk to flash memory
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_END) {
    // Finalize the update and verify file size/integrity
    if (Update.end(true)) {
      Serial.printf("[OTA] Success: %u bytes. System will reboot.\n", upload.totalSize);
    } else {
      Update.printError(Serial);
    }
  }
}


/*
 * Endpoint for calucation request
 *  -GET
 *  -ADD / POST
 *  -DELETE
 */



/* Summary of function: Handles POST requests using a switch statement for type-specific validation and mapping */
void handleCalculationSave() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  int8_t type = (int8_t)server.arg("type").toInt();
  String expression = server.arg("expression");
  String tag = server.arg("tag");
  String error;

  // Initialize a fresh config object
  CalculationConfig newCalc = {};
  newCalc.type = type;
  strlcpy(newCalc.name, server.arg("name").c_str(), sizeof(newCalc.name));
  strlcpy(newCalc.unit, server.arg("unit").c_str(), sizeof(newCalc.unit));
  newCalc.publishValue = (server.arg("publishValue") == "1");

  // Type-specific validation and mapping
  switch (newCalc.type) {
    case CALC_EXPRESSION:
      if (!expressionManager.validateExpression(expression, error)) { server.send(400, "text/plain", "Input Error: " + error); return; }
      strlcpy(newCalc.expression.expression, expression.c_str(), sizeof(newCalc.expression.expression));
      newCalc.expression.triggerThreshold = server.arg("triggerThreshold").toFloat();
      break;

    case CALC_CONTINUOUS:
      if (!expressionManager.validateExpression(expression, error)) { server.send(400, "text/plain", "Input Error: " + error); return; }
      strlcpy(newCalc.continuous.expression, expression.c_str(), sizeof(newCalc.continuous.expression));
      newCalc.continuous.evalIntervalMs = server.arg("interval").toInt();
      newCalc.continuous.minValue = (server.arg("min") == "") ? NAN : server.arg("min").toFloat();
      newCalc.continuous.maxValue = (server.arg("max") == "") ? NAN : server.arg("max").toFloat();
      break;

    case CALC_EXTERNAL:
      /* Summary of logic: Validate that the tag is either a local resource or a valid remote MQTT path */
      if (!validateBindingTag(tag, error)) {
        server.send(400, "text/plain", "Binding Error: " + error);
        return;
      }
      strlcpy(newCalc.binding.tag, tag.c_str(), sizeof(newCalc.binding.tag));
      newCalc.binding.fallbackValue = server.arg("fallbackValue").toFloat();
      newCalc.binding.resetIntervalMs = server.arg("resetIntervalMs").toInt();
      newCalc.binding.isResetIntervalRef = false;
      break;

    default:
      server.send(400, "text/plain", "Invalid Calculation Type");
      return;
  }

  bool success = false;
  if (server.hasArg("calcId") && server.arg("calcId") != "") {
    int id = server.arg("calcId").toInt();
    if (id >= 0 && id < MAX_CALCULATIONS_ALLOWED) {
      expressionManager.updateCalculation(id, newCalc, mqtt);
      success = true;
    }
  } else {
    int resultIndex = expressionManager.addCalculation(newCalc, mqtt);
    success = (resultIndex >= 0);
  }

  if (success) {
    saveConfig();
    if (server.arg("responseFormat") == "html") {
      server.send(200, "text/html", displayCalculationsInProduction());
    } else {
      server.sendHeader("Location", "/#configuredCalculations");
      server.send(303);
    }
  } else {
    server.send(500, "text/plain", "Storage Error: No slots available");
  }
}

void handleCalculationDelete() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  int id = server.arg("id").toInt();
  expressionManager.removeCalculation(id, mqtt);
  saveConfig();

  if (server.arg("responseFormat") == "html") {
    server.send(200, "text/html", displayCalculationsInProduction());
  } else {
    server.send(200, "text/plain", "Deleted");
  }
}


/* Summary of function: Handles POST requests for actuators using a switch statement for type-specific mapping */
void handleActuatorSave() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  // Initialize a fresh config object
  ActuatorConfig newAct = {};
  newAct.type = (ActuatorType)server.arg("type").toInt();

  strlcpy(newAct.name, server.arg("name").c_str(), sizeof(newAct.name));
  strlcpy(newAct.sourceName, server.arg("sourceName").c_str(), sizeof(newAct.sourceName));

  // Publish policy from form select
  uint8_t triggerVal = (uint8_t)server.arg("publishTrigger").toInt();
  newAct.publishPolicy.trigger = (triggerVal <= PUBLISH_ON_INTERVAL)
                                 ? (PublishTrigger)triggerVal
                                 : PUBLISH_ON_THRESHOLD_OR_INTERVAL;

  // Type-specific mapping using switch statement
  switch (newAct.type) {
    case ACTUATOR_DIGITAL:
      newAct.digital.digitalPin = (uint8_t)server.arg("digitalPin").toInt();
      if (server.hasArg("inverted")) {
        newAct.digital.inverted = (server.arg("inverted") == "1");
      } else {
        newAct.digital.inverted = false;
      }
      break;

    case ACTUATOR_ANALOG:
      newAct.analog.analogPin = (uint8_t)server.arg("analogPin").toInt();
      newAct.analog.minValue = server.arg("minValue").toFloat();
      newAct.analog.maxValue = server.arg("maxValue").toFloat();
      newAct.analog.voltageMin = server.arg("voltageMin").toFloat();
      newAct.analog.voltageMax = server.arg("voltageMax").toFloat();
      strlcpy(newAct.analog.unit, server.arg("unit").c_str(), sizeof(newAct.analog.unit));
      break;

    default:
      server.send(400, "text/plain", "Invalid Actuator Type");
      return;
  }

  int finalId = -1; // To track which slot was actually modified
  bool success = false;

  if (server.hasArg("actId") && server.arg("actId") != "") {
    finalId = server.arg("actId").toInt();
    if (finalId >= 0 && finalId < MAX_ACTUATORS_ALLOWED) {
      memcpy(&config.actuators[finalId], &newAct, sizeof(ActuatorConfig));
      success = true;
    }
  } else {
    finalId = actuatorManager.addActuator(newAct); // Manager returns the index used
    success = (finalId >= 0);
  }

  if (success) {
    // CRITICAL FIX: Use finalId to reset the runtime state
    // This forces updateActuators to skip the 'persistOnlyChange' check
    // and apply the inversion logic on the next loop iteration.
    config.actuators[finalId].runtimeValue = -999.0f;

    saveConfig();

    if (server.arg("responseFormat") == "html") {
      server.send(200, "text/html", displayActuatorsInProduction());
    } else {
      String redirectComponent = "#acturatorHeader";
      server.sendHeader("Location", String("/") + redirectComponent);
      server.send(303);
    }
  } else {
    server.send(500, "text/plain", "Storage Error: No actuator slots available");
  }
}


/* Summary of function: Handles Actuator deletion and returns updated HTML table fragment */
void handleActuatorDelete() {
  SaveConfigManager mgr(config);
  if (!mgr.authenticateRequest()) {
    server.send(401, "text/plain", "Unauthorized");
    return;
  }

  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Missing ID");
    return;
  }

  int id = server.arg("id").toInt();
  if (id >= 0 && id < MAX_ACTUATORS_ALLOWED) {
    // Reset slot to unconfigured state
    config.actuators[id].type = ACTUATOR_UNCONFIGURED;
    memset(config.actuators[id].name, 0, sizeof(config.actuators[id].name));
    memset(config.actuators[id].sourceName, 0, sizeof(config.actuators[id].sourceName));

    saveConfig();
  }

  if (server.arg("responseFormat") == "html") {
    server.send(200, "text/html", displayActuatorsInProduction());
  } else {
    server.send(200, "text/plain", "Deleted");
  }
}









void handleWinkRequest() {
  // visualizationOfError[WINK];

  String state = server.arg("state"); // les ?state=on/off
  if (state == "on") {
    digitalWrite(ONBOARD_LED, LOW);   // LED ON (ESP8266 aktiv lav)
    winkActive = true;
    server.send(200, "text/plain", "LED turned ON");
  } else if (state == "off") {
    digitalWrite(ONBOARD_LED, HIGH);  // LED OFF
    winkActive = false;
    server.send(200, "text/plain", "LED turned OFF");
  } else {
    server.send(400, "text/plain", "Invalid state");
  }
}


// Web server handlers for backup/restore
// Web server handler for backup as downloadable file
void handleBackupRequest() {
  if (DEBUG) {
    Serial.println("=== START BACKUP BASE64 DOWNLOAD FLOW ===");
  }

  // 1. Generate Base64 JSON from current EEPROMLayout
  String backupJson = BackupManager::generateBackupJsonBase64();
  if (DEBUG) {
    Serial.println("Generated backup JSON (Base64):");
    Serial.println(backupJson);
  }

  String filename = generateAPName() + "_backup"; // May be adding clock from broker later

  // 2. Send JSON as a downloadable file
  server.sendHeader("Content-Type", "application/json");
  server.sendHeader("Content-Disposition", "attachment; filename=\"backup.json\"");
  server.send(200, "application/json", backupJson);

  if (DEBUG) {
    Serial.println("Backup JSON sent as downloadable file");
    Serial.println("=== END BACKUP BASE64 DOWNLOAD FLOW ===");
  }
}




void handleBackupUpload() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  HTTPUpload& upload = server.upload();
  static String fileContent;

  if (upload.status == UPLOAD_FILE_START) {
    Serial.println("=== START BACKUP UPLOAD ===");
    fileContent = "";
    Serial.print("Uploading file: ");
    Serial.println(upload.filename);
  }
  else if (upload.status == UPLOAD_FILE_WRITE) {
    // Append bytes from upload buffer
    for (size_t i = 0; i < upload.currentSize; i++) {
      fileContent += (char)upload.buf[i];
    }
  }
  else if (upload.status == UPLOAD_FILE_END) {
    Serial.println("Upload complete, restoring backup...");

    bool restoreSuccess = BackupManager::restoreFromBackupJsonBase64(fileContent);
    if (restoreSuccess) {
      Serial.println("EEPROM restored successfully from uploaded backup!");

      // Send response with JS redirect after short delay
      String response = "Backup restored successfully. Device will restart in 3 seconds...";
      response += "<script>setTimeout(function(){window.location='/'},3000);</script>";
      server.send(200, "text/html", response);

      delay(3000); // la klienten se meldingen
      ESP.restart(); // restart ESP
    } else {
      Serial.println("Failed to restore EEPROM from uploaded backup.");
      server.send(500, "text/plain", "Failed to restore backup. Check console for details.");
    }

    Serial.println("=== END BACKUP UPLOAD ===");
  }
}




/*

  // ---------------------------
  // Upload a new certificate + key
  // ---------------------------
  void handleCertificateUpload() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  if (!server.hasArg("certfile") || !server.hasArg("keyfile")) {
    server.send(400, "text/plain", "Missing certificate or key file");
    return;
  }

  // Read uploaded certificate and key from request
  String certStr = server.arg("certfile");  // In POST handler, file content may come differently depending on your upload library
  String keyStr = server.arg("keyfile");

  if (certStr.length() == 0 || keyStr.length() == 0) {
    server.send(400, "text/plain", "Empty certificate or key");
    return;
  }

  String result;
  if (strlen(config.certificates.cert) > 0) {
    result = certificateManager.updateCertificate(certStr.c_str(), keyStr.c_str());
  } else {
    result = certificateManager.addCertificate(certStr.c_str(), keyStr.c_str());
  }

  certificateManager.saveChanges();
  server.send(200, "text/plain", result);
  }

  // ---------------------------
  // Delete existing certificate
  // ---------------------------
  void handleCertificateDelete() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  String result = certificateManager.removeCertificate();
  certificateManager.saveChanges();
  server.send(200, "text/plain", result);
  }

  // ---------------------------
  // Validate stored certificate
  // ---------------------------
  void handleCertificateValidate() {
  if (!isAuthenticated()) {
    server.requestAuthentication();
    return;
  }

  bool valid = certificateManager.validateCertificate();
  String result = valid ? "Certificate is valid." : "Certificate is invalid or missing.";

  server.send(200, "text/plain", result);
  }



*/







/*
   Main server boilerplate
    Wifi, webserver, readings etc
*/

void debugSystemStatus() {
  #if defined(ESP8266)

  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t maxFreeBlock = ESP.getMaxFreeBlockSize();
  uint8_t fragmentation = ESP.getHeapFragmentation();

  float ramTotal = 81920.0;
  float ramUsedPercent = 100.0f * (ramTotal - freeHeap) / ramTotal;

  uint32_t uptimeMs = millis();

  Serial.printf("Free heap: %u bytes\n", freeHeap);
  Serial.printf("Max free block: %u bytes\n", maxFreeBlock);
  Serial.printf("Heap fragmentation: %u percent\n", fragmentation);
  Serial.printf("RAM used percent: %.2f\n", ramUsedPercent);
  Serial.printf("Uptime ms: %u\n", uptimeMs);
  Serial.println("Temperature sensor not available on ESP8266");

  #elif defined(ESP32)

  // Heap
  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minFreeHeap = ESP.getMinFreeHeap();
  uint32_t ramTotal = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  float ramUsedPercent = 100.0f * (ramTotal - freeHeap) / ramTotal;

  // CPU frekvens
  uint32_t cpuFreq = ESP.getCpuFreqMHz();

  // Chip info via Arduino API
  Serial.printf("Chip model: %s\n", ESP.getChipModel());
  Serial.printf("Chip revision: %d\n", ESP.getChipRevision());
  Serial.printf("Chip cores: %d\n", ESP.getChipCores());

    #if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
  int tempC = temperatureRead();
  Serial.printf("Temperature C: %d\n", tempC);
    #else
  Serial.println("Temperature sensor not available on this ESP32 target");
    #endif

  // Uptime
  uint64_t uptimeUs = esp_timer_get_time();

  Serial.printf("Free heap: %u bytes\n", freeHeap);
  Serial.printf("Minimum free heap: %u bytes\n", minFreeHeap);
  Serial.printf("RAM used percent: %.2f\n", ramUsedPercent);
  Serial.printf("CPU frequency MHz: %u\n", cpuFreq);
  Serial.printf("Uptime us: %llu\n", uptimeUs);

  // FreeRTOS stack status
  TaskHandle_t task = xTaskGetCurrentTaskHandle();
  uint32_t stackHighWater = uxTaskGetStackHighWaterMark(task);
  Serial.printf("Stack high water mark: %u bytes\n", stackHighWater);

  #endif
}






bool connectWiFi() {
  // Use the GLOBAL config, not a local one!
  if (strlen(config.network.wifiSSID) == 0) {
    Serial.println("No WiFi credentials configured");
    return false;
  }
  Serial.printf("\nAttempting to connect to: %s\n", config.network.wifiSSID);


  #if DEBUG_NETWORK_CONNECTION
  Serial.println("Debug network diagnostics enabled");
  Serial.printf("Password length: %u\n", strlen(config.network.wifiPassword));
  Serial.printf("Stored password: %s\n", config.network.wifiPassword);
  #endif

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(config.network.wifiSSID, config.network.wifiPassword);


  int attempts = 0;
  const int maxAttempts = 20;
  bool connected = false;

  while (attempts < maxAttempts) {
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      break;
    }
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (connected) {
    Serial.println("\nConnected successfully!");
    Serial.printf("SSID: %s\n", WiFi.SSID().c_str());
    Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());

    #if DEBUG_NETWORK_CONNECTION
    Serial.printf("BSSID: %s\n", WiFi.BSSIDstr().c_str());
    Serial.printf("Channel: %d\n", WiFi.channel());
    Serial.printf("RSSI: %d\n", WiFi.RSSI());
    Serial.printf("DNS1: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("DNS2: %s\n", WiFi.dnsIP(1).toString().c_str());
    #endif

    String hostname = generateAPName();
    if (MDNS.begin(hostname.c_str())) {
      Serial.printf("mDNS responder started: %s.local\n", hostname.c_str());
    } else {
      Serial.println("Error setting up mDNS responder!");
    }

    return true;
  } else {
    Serial.println("\nConnection failed. Status:");
    printWiFiStatus();

    #if DEBUG_NETWORK_CONNECTION
    Serial.printf("Final WiFi status code: %d\n", WiFi.status());
    Serial.printf("Last known RSSI: %d\n", WiFi.RSSI());
    Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("Diagnostics complete");
    #endif

    return false;
  }
}

#if !WIRELESS_NETWORK
#include <Ethernet.h>

bool connectEthernet() {
  String apName = generateAPName();
  const uint8_t* macConst = generateMacFromAP(apName.c_str());

  uint8_t mac[6];                 // mutable copy
  memcpy(mac, macConst, 6);       // copy the 6 bytes

  Serial.printf("\nAttempting to start Ethernet with MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  const int maxAttempts = 20;
  int attempts = 0;
  bool connected = false;

  if (config.network.useDHCP) {
    Serial.println("Using DHCP for Ethernet...");
    Ethernet.begin(mac);
  } else {
    Serial.println("Using static IP for Ethernet...");
    Ethernet.begin(mac,
                   config.network.staticIP,
                   config.network.gateway,
                   config.network.gateway,
                   config.network.subnet);
  }

  while (attempts < maxAttempts) {
    IPAddress ip = Ethernet.localIP();
    if (ip != (uint32_t)0) {
      connected = true;
      break;
    }
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (!connected) {
    Serial.println("\nEthernet connection failed");
    return false;
  }

  IPAddress ip = Ethernet.localIP();
  Serial.println("\nConnected successfully!");
  Serial.print("IP Address: ");
  Serial.println(ip);

  return true;
}
#endif


#define DEBUG_ONMESSAGE true

/* Summary of function: Global callback that triggers whenever a message arrives.
   Converts payload to string, prints debug info if enabled, and saves to MqttManager cache. */
void OnMessage(char* topic, byte* payload, unsigned int length) {
  // 1. Create a null-terminated string from the payload
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';

  // 2. Debugging output
  #if DEBUG_ONMESSAGE
  Serial.println("--- MQTT Inbound ---");
  Serial.printf("Topic:  %s\n", topic);
  Serial.printf("Len:    %u\n", length);
  Serial.printf("Data:   %s\n", message);
  Serial.println("--------------------");
  #endif

  // 3. Save exclusively to RAM cache in MqttManager
  mqtt.handleIncomingPayload(topic, message);
}


void setupServerRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config/wifi", HTTP_POST, handleWifiSettingsChange);
  server.on("/config/ethernet", HTTP_POST, handleEthernetSettingsChange);
  server.on("/config/mqtt", HTTP_POST, handleMQTTSettingsChange);
  server.on("/config/operations", HTTP_POST, handleOperationsSettingsChange);
  server.on("/favicon.svg", HTTP_GET, []() {
    String svg = getLogoSvg();
    server.sendHeader("Cache-Control", "public, max-age=86400");
    server.send(200, "image/svg+xml", svg);
  });
  server.on("/config/sensor/types", HTTP_GET, handleSensorTypeRequest);
  server.on("/config/sensors", HTTP_GET, handleGetAllSensors);
  server.on("/config/sensor", HTTP_POST, handleSensorConfig);
  server.on("/config/sensor", HTTP_DELETE, handleDeleteSensor);
  server.on("/config/device", HTTP_POST, handleDeviceInfo);
  server.on("/config/sensor/edit", HTTP_GET, handleGetEditPage);

  // Save calclation
  server.on("/config/calculation", HTTP_POST, handleCalculationSave);
  server.on("/config/calc/delete", handleCalculationDelete);
  server.on("/config/calc/edit", HTTP_GET, handleGetEditCalculationPage);

  // Actuator Configuration Routes
  server.on("/config/actuator", HTTP_POST, handleActuatorSave);
  server.on("/config/actuator/delete", HTTP_GET, handleActuatorDelete);
  server.on("/config/actuator/edit", HTTP_GET, handleGetEditActuatorPage);

  server.on("/firmware", HTTP_GET, firmwareVersionPage);
  server.on("/firmware", HTTP_POST, []() {
    if (Update.hasError()) {
      server.send(500, "text/plain", String("Update Failed: ") + Update.errorString());
    } else {
      // Send a response to the browser before the ESP reboots
      String successHtml = "<html><body style='font-family:sans-serif;text-align:center;padding-top:50px;'>";
      successHtml += "<h1>Update Success!</h1><p>Grefur-sensor is rebooting...</p>";
      successHtml += "<script>setTimeout(function(){ window.location.href = '/'; }, 5000);</script>";
      successHtml += "</body></html>";

      server.send(200, "text/html", successHtml);

      // Short delay to ensure the HTTP response packet is sent
      delay(1000);
      ESP.restart();
    }
  }, firmwareUploadPage); // firmwareUploadPage handles the actual data chunks

  // Wink routes
  server.on("/config/wink", HTTP_GET, handleWinkRequest);

  // Backup routes
  server.on("/config/backup", HTTP_GET, backupPage);        // HTML
  server.on("/backup/file", HTTP_GET, handleBackupRequest); // BBackup download endpoint
  server.on("/backup/upload", HTTP_POST, []() {
    server.send(200, "text/plain", "Upload complete");
  }, handleBackupUpload); // Backup opplasting

  server.on("/certificate", HTTP_GET, handleCertificatePages);
  //server.on("/certificate/upload", HTTP_POST, handleCertificateUpload);
  //server.on("/certificate/delete", HTTP_POST, handleCertificateDelete);
  //server.on("/certificate/validate", HTTP_POST, handleCertificateValidate);


}


void beginMicroControllerServer() {
#if defined(ESP8266)
  server.begin();

#elif defined(ESP32)
  server.begin();
  httpUpdater.setup(&server);

#elif defined(STM32F7) || defined(STM32H7)
  server.begin();

#else
    #error "Unsupported board!"
#endif
}

// IP: 192.168.4.1

void startConfigPortal() {
  String apName = generateAPName();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName.c_str());

  Serial.println("Started configuration portal");
  Serial.println("AP Name: " + apName);
  Serial.println("IP address: " + WiFi.softAPIP().toString());
  visualizationOfError(GENERAL_ERROR);



  setupServerRoutes();
  beginMicroControllerServer();

}

MqttManager mqtt(mqttClient);


/* Summary of function: Sets factory defaults, generates hardware ID, and
   initializes sub-systems. Minimalist version focused on identity and Grefur flag. */
void initializeDefaultConfigValues() {
  // 1. Wipe the RAM structure
  memset(&config, 0, sizeof(config));

  // 2. Set static header information
  config.header.magic = 0xAA55AA55;
  strncpy(config.header.version, FIRMWARE_VERSION, sizeof(config.header.version));

  // 3. Generate hardware Fingerprint (Critical for identity)
  uint32_t tokenNum = generateDeviceToken();
  snprintf(config.device.productionToken, 9, "%08X", tokenNum);

    #if (DEBUG_SYSTEM)
  Serial.printf("[SYSTEM] Initializing Grefur Sensor: %s\n", config.device.productionToken);
    #endif

  // 4. Initialize sensor hardware slots
  sensorManager.initializeSensorSlots();
  actuatorManager.initializeActuatorSlots();

  // 5. Use SaveConfigManager for state flags
  SaveConfigManager mgr(config);
  mgr.setSendHtml(false);

  // Enable Grefur Backend by default
  mgr.updateBoolField(config.mqtt.useGrefurBackend, true);

  // Operation settings: Replaced server.arg with sensible default values for "Grefur-Sensor"
  // Using 60000ms (1 min) as a standard publish interval
  mgr.updateNumericField(config.operation.publishInterval, "60000");
  mgr.updateBoolField(config.operation.publishStatus, true);
  mgr.updateBoolField(config.operation.publishDeviceInfo, true);

  // Set basic metadata
  mgr.updateStringField(config.device.deviceType, sizeof(config.device.deviceType), "Backplate");
  // Set Device Name
  mgr.updateStringField(config.device.deviceName, sizeof(config.device.deviceName), "Grefur-Sensor");
  // Set default baseTopic to ensure grefur topology
  mgr.updateStringField(config.mqtt.baseTopic, sizeof(config.mqtt.baseTopic), "default");

  // 6. Persist to Flash
  mgr.scheduleRestart();
  mgr.finalize();

}



void setup() {
  Serial.begin(115200);
  Serial.println("\nStarting Grefur Sensors v" + getFirmwareVersion());

  EEPROM.begin(sizeof(EEPROMLayout));
  bool loadDefault = false;

  #if defined(ESP32)
  analogReadResolution(12);
  delay(100);
  #endif

  if (!loadConfig() || loadDefault) {
    Serial.println("Invalid config or first boot detected.");
    initializeDefaultConfigValues();
    visualizationOfError(GENERAL_ERROR);
  }

  Serial.printf("Fingerprint: %s\n", config.device.productionToken);

  BackupManager::begin(); // Henter EEPROM eller setter default

  //setupOTA();

  //certificateManager.begin();
  // Generate self-signed cert if none exists
  //if (!certificateManager.validateCertificate()) {
  //certificateManager.generateSelfSignedCert(generateAPName(), 365);
  //}

  //Serial.println(certificateManager.getCertificateSummary());


  pinMode(ONBOARD_LED, OUTPUT);
  //digitalWrite(ONBOARD_LED, ONBOARD_LED_OFF);

  //pinMode(MUX_A, OUTPUT);
  //pinMode(MUX_B, OUTPUT);

  //testConfigIntegrity();
  //debugSystemStatus();



  // WiFi / Ethernet initialization
#if WIRELESS_NETWORK
  if (connectWiFi()) {
    mqttClient.setServer(config.mqtt.brokerURL, config.mqtt.port);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(30);


    setupServerRoutes();

    beginMicroControllerServer();

    Serial.println("HTTP server using WiFi started");

  } else {
    Serial.println("Config portal started...");
    startConfigPortal();
  }

#else
  if (connectEthernet()) {
    mqttClient.setServer(config.mqtt.brokerURL, config.mqtt.port);
    mqttClient.setKeepAlive(60);
    mqttClient.setSocketTimeout(30);

    setupServerRoutes();
    Serial.println("HTTP server using Ethernet started");
  }
#endif
}


/* Summary of function: Main execution loop handling autonomous logic, sensor sampling, and network communication. */
void loop() {
  // 1. Core System Maintenance
  server.handleClient();
  //ArduinoOTA.handle();
  updateErrorVisualization();

  unsigned long currentTime = millis();
  static unsigned long lastLoopTime = 0;
  static unsigned long lastMeasurementTime = 0;
  static unsigned long lastPublishTime = 0;
  static unsigned long lastMQTTReconnectAttempt = 0;
  static unsigned long lastWifiCheck = 0;

  uint32_t deltaTime = currentTime - lastLoopTime;
  lastLoopTime = currentTime;

  #define TICK_TIME 50

  // 2. Continuous Sensor Sampling (Independent of Network)
  if (currentTime - lastMeasurementTime >= TICK_TIME) {
    lastMeasurementTime = currentTime;
    float currentReadings[MAX_SENSORS_ALLOWED];
    takeMeasurements(currentReadings);

    for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
      if (!isnan(currentReadings[i])) {
        sensorManager.setCurrentReading(i, currentReadings[i]);
      }
    }

    // 3. Logic & Actuation (The Engine)
    expressionManager.evaluateAll(deltaTime);
    actuatorManager.updateActuators(expressionManager);
  }




  // 4. Network & Communication Block
  if (WiFi.status() == WL_CONNECTED) {
        #if defined(ESP8266)
    MDNS.update();
        #endif

    // Manage MQTT Connection
    if (!mqttClient.connected()) {
      if (currentTime - lastMQTTReconnectAttempt > 10000) {
        lastMQTTReconnectAttempt = currentTime;
        if (mqtt.reconnect()) {
          expressionManager.registerExternalSubscriptions(mqtt);
          mqttClient.setCallback(OnMessage);
        }
      }
    } else {
      mqttClient.loop();
    }

    // Scheduled Data Publishing
    if (currentTime - lastPublishTime >= config.operation.publishInterval) {
      lastPublishTime = currentTime;

      // Publish physical sensor values
      for (int i = 0; i < MAX_SENSORS_ALLOWED; i++) {
        if (!sensorManager.isSensorOfIndexConfigured(i)) continue;

        // this irritation value
        float val = sensorManager.getCurrentReading(i);

        if (isnan(val)) continue;

        const SensorConfig& sensor = sensorManager.getSensorConfig(i);
        float delta = abs(val - sensor.lastMeasurement);
        Serial.print("Delta: ");
        Serial.println(delta);

        // Milliseconds since last publishment of this value was made
        uint32_t elapsed = currentTime - sensor.lastUpdateTs;
        bool shouldPublish = false;

        switch (sensor.publishPolicy.trigger) {
          case PUBLISH_ON_THRESHOLD_OR_INTERVAL:
            shouldPublish = (delta >= sensor.publishPolicy.changeThreshold)
                            || (elapsed >= sensor.publishPolicy.intervalMs);
            break;
          case PUBLISH_ON_CHANGE_OF_THRESHOLD:
            shouldPublish = (delta >= sensor.publishPolicy.changeThreshold);
            break;
          case PUBLISH_ON_INTERVAL:
            shouldPublish = (elapsed >= sensor.publishPolicy.intervalMs);
            break;
          default:
            shouldPublish = true;
            break;
        }

        #if DEBUG_PUBLISH
        Serial.printf("[Sensor %d] val=%.2f delta=%.2f elapsed=%ums threshold=%.2f intervalMs=%u trigger=%d -> %s\n",
                      i, val, delta, elapsed,
                      sensor.publishPolicy.changeThreshold,
                      sensor.publishPolicy.intervalMs,
                      sensor.publishPolicy.trigger,
                      shouldPublish ? "PUBLISH" : "SKIP");
        #endif

        if (shouldPublish) {
          mqtt.publishMeasuredValue(val, sensorManager.getSensorName(i).c_str(),
                                    sensorManager.getSensorUnit(i).c_str(),
                                    sensorManager.getSensorType(i));

          sensorManager.setLastMeasurement(i, val);
          sensorManager.setLastUpdateTs(i, currentTime);
        }
      }

      /* Summary of function:
                                                                                                                         *  Iterates through calculated expression results and publishes them to MQTT.
                                                                                                                         *  Includes a safety check to ensure the calculation has a valid name to prevent malformed MQTT topics.
                                                                                                                         */
      for (int i = 0; i < MAX_CALCULATIONS_ALLOWED; i++) {
        CalculationConfig& cfg = config.calculations[i];
        if (cfg.type > 0 && cfg.publishValue && strlen(cfg.name) > 0) {
          float result = expressionManager.GetLastValue(i);

          if (!isnan(result)) {
            mqtt.publishMeasuredValue(result, cfg.name, cfg.unit, 10);
          }
        }
      }

      /* Summary of function:
                                                                                                                                                                                                                                       Publishes current hardware actuator states to the MQTT broker.
                                                                                                                                                                                                                                       Dynamically retrieves the name, value, unit, and type to ensure the Grefur UI renders the correct component. */
      for (int i = 0; i < MAX_ACTUATORS_ALLOWED; i++) {
        if (!actuatorManager.shouldPublishActuator(i, currentTime)) continue;
        float actVal = actuatorManager.getValue(i);
        const char* actName = config.actuators[i].name;
        String unit = actuatorManager.getUnit(i);
        int type = actuatorManager.getType(i);
        mqtt.publishMeasuredValue(actVal, actName, unit.c_str(), type);
        actuatorManager.setLastPublished(i, actVal, currentTime);
      }
    }

  } else {
    // Handle WiFi Disconnection without blocking local logic
    if (currentTime - lastWifiCheck > 10000) {
      lastWifiCheck = currentTime;
      Serial.println("WiFi disconnected, attempting reconnect...");
      WiFi.reconnect();
    }
    visualizationOfError(NO_INTERNET);
  }

  // 5. System Health
  if (!errorState.isActive) {
    visualizationOfError(SYSTEM_OK);
  }

  delay(10); // Breathing room for the network stack
}
