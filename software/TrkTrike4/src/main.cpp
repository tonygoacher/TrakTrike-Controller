
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <stddef.h>
#include <math.h>
#include <Adafruit_MCP4728.h>
#include "ports.h"

// =====================
// MCP4728
// =====================
Adafruit_MCP4728 mcp;

// =====================
// CONFIG
// =====================
#define EEPROM_ADDR 0
#define PARAM_VERSION 1

typedef struct {
    uint8_t  version;
    uint16_t crc;
} config_header_t;

struct Config {
    config_header_t header;

    int DAC_MIN;
    int DAC_START;
    int DAC_MAX;

    float THROTTLE_DEADBAND;
    float TAKEUP_END;

    float RAMP_UP_RATE;
    float RAMP_DOWN_RATE;

    float trim;

    int THROTTLE_MIN_ADC;
    int THROTTLE_MAX_ADC;
};

// =====================
// RUNTIME PARAMS
// =====================
int DAC_MIN;
int DAC_START;
int DAC_MAX;

float THROTTLE_DEADBAND;
float TAKEUP_END;

float RAMP_UP_RATE;
float RAMP_DOWN_RATE;

float trim;

int THROTTLE_MIN_ADC;
int THROTTLE_MAX_ADC;

// =====================
// THROTTLE CLASS
// =====================
class TwistThrottle {
public:
    struct Config {
        int minADC;
        int maxADC;
        int deadband;
        float filterAlpha;
        float maxRate;
    };

    void begin(const Config& cfg) {
        config = cfg;
        filtered = config.minADC;
        lastOutput = 0.0f;
        lastTime = millis();
    }

    float GetThrottle() {

        int raw = analogRead(THROTTLE);
        //Serial.println(raw);

        int validMin = config.minADC - 20;
        int validMax = config.maxADC + 20;

        if (raw < validMin || raw > validMax) 
        { 
            fault = true; return 0.0f; 
        }

        fault = false;

        filtered += (raw - filtered) * config.filterAlpha;

        if (filtered < (config.minADC + config.deadband)) {
            return applyRateLimit(0.0f);
        }

        float output = (filtered - (config.minADC + config.deadband)) /
                       (float)(config.maxADC - (config.minADC + config.deadband));

        output = constrain(output, 0.0f, 1.0f);


    //    Serial.print("  OUT: ");
      //  Serial.println(output, 4);

        return applyRateLimit(output);
    }

    bool isFault() const {
        return fault;
    }

private:
    Config config;

    float filtered = 0.0f;
    float lastOutput = 0.0f;
    unsigned long lastTime = 0;

    bool fault = false;

    float applyRateLimit(float target) {

        if (config.maxRate <= 0.0f)
            return target;

        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        lastTime = now;

        float maxDelta = config.maxRate * dt;
        float delta = target - lastOutput;

        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;

        lastOutput += delta;

        return lastOutput;
    }
};

TwistThrottle throttle;

// =====================
// CALIBRATION STATE
// =====================
struct ThrottleCal {
    int minADC = 1023;
    int maxADC = 0;
};

ThrottleCal throttleCal;

// =====================
// STATE
// =====================
float currentOutput = 0.0f;
String inputString = "";

// =====================
// FORWARD DECLARATIONS
// =====================
void processCommand(String cmd);
void updateDACDefaults();
void applyCalibration();
void configureThrottle();
void printParams();

void printConfig(const Config& cfg)
{
    Serial.println("===== CONFIG =====");

    Serial.print("Version: ");
    Serial.println(cfg.header.version);

    Serial.print("CRC: ");
    Serial.println(cfg.header.crc);

    Serial.println();

    Serial.print("DAC_MIN: ");
    Serial.println(cfg.DAC_MIN);

    Serial.print("DAC_START: ");
    Serial.println(cfg.DAC_START);

    Serial.print("DAC_MAX: ");
    Serial.println(cfg.DAC_MAX);

    Serial.println();

    Serial.print("THROTTLE_DEADBAND: ");
    Serial.println(cfg.THROTTLE_DEADBAND, 4);

    Serial.print("TAKEUP_END: ");
    Serial.println(cfg.TAKEUP_END, 4);

    Serial.println();

    Serial.print("RAMP_UP_RATE: ");
    Serial.println(cfg.RAMP_UP_RATE, 4);

    Serial.print("RAMP_DOWN_RATE: ");
    Serial.println(cfg.RAMP_DOWN_RATE, 4);

    Serial.println();

    Serial.print("trim: ");
    Serial.println(cfg.trim, 4);

    Serial.println();

    Serial.print("THROTTLE_MIN_ADC: ");
    Serial.println(cfg.THROTTLE_MIN_ADC);

    Serial.print("THROTTLE_MAX_ADC: ");
    Serial.println(cfg.THROTTLE_MAX_ADC);

    Serial.println("==================");
}

// =====================
// CRC
// =====================
uint16_t calculateCRC(const Config &cfg) {
    const uint8_t *data = (const uint8_t*)&cfg;

    size_t start = offsetof(Config, DAC_MIN);
    size_t len   = sizeof(Config) - start;

    uint16_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        crc += data[start + i];
    }

    return crc;
}

// =====================
// CONFIG APPLY
// =====================
void applyConfig(const Config &cfg) {

    DAC_MIN = cfg.DAC_MIN;
    DAC_START = cfg.DAC_START;
    DAC_MAX = cfg.DAC_MAX;

    THROTTLE_DEADBAND = cfg.THROTTLE_DEADBAND;
    TAKEUP_END = cfg.TAKEUP_END;

    RAMP_UP_RATE = cfg.RAMP_UP_RATE;
    RAMP_DOWN_RATE = cfg.RAMP_DOWN_RATE;

    trim = cfg.trim;

    THROTTLE_MIN_ADC = cfg.THROTTLE_MIN_ADC;
    THROTTLE_MAX_ADC = cfg.THROTTLE_MAX_ADC;

    printConfig(cfg);
}

// =====================
// CONFIG SAVE/LOAD
// =====================
void saveConfig() {
    Config cfg;

    cfg.header.version = PARAM_VERSION;

    cfg.DAC_MIN = DAC_MIN;
    cfg.DAC_START = DAC_START;
    cfg.DAC_MAX = DAC_MAX;

    cfg.THROTTLE_DEADBAND = THROTTLE_DEADBAND;
    cfg.TAKEUP_END = TAKEUP_END;

    cfg.RAMP_UP_RATE = RAMP_UP_RATE;
    cfg.RAMP_DOWN_RATE = RAMP_DOWN_RATE;

    cfg.trim = trim;

    cfg.THROTTLE_MIN_ADC = THROTTLE_MIN_ADC;
    cfg.THROTTLE_MAX_ADC = THROTTLE_MAX_ADC;

    cfg.header.crc = calculateCRC(cfg);

    EEPROM.put(EEPROM_ADDR, cfg);

    Serial.println("Saved config");
}

bool loadConfig() {
    Config cfg;
    EEPROM.get(EEPROM_ADDR, cfg);

    if (cfg.header.version != PARAM_VERSION) 
    {
          Serial.println("Unknown config version");
          return false;
    }

    if (calculateCRC(cfg) != cfg.header.crc)
    {
      Serial.println("Config load failed CRC");
      return false;
    }      

    applyConfig(cfg);

    Serial.println("Loaded config");
    return true;
}

void loadDefaults() {

    DAC_MIN = 800;
    DAC_START = 800;
    DAC_MAX = 3500;

    THROTTLE_DEADBAND = 0.05f;
    TAKEUP_END = 0.20f;

    RAMP_UP_RATE = 0.05f;
    RAMP_DOWN_RATE = 0.02f;

    trim = 0.0f;

    THROTTLE_MIN_ADC = 0;
    THROTTLE_MAX_ADC = 1023;

    Serial.println("Loaded defaults");
    printParams();
    
}

// =====================
// THROTTLE SETUP
// =====================
void configureThrottle() {

    TwistThrottle::Config cfg;

    cfg.minADC = THROTTLE_MIN_ADC;
    cfg.maxADC = THROTTLE_MAX_ADC;

    cfg.deadband = 8;
    cfg.filterAlpha = 0.1f;
    cfg.maxRate = 0.0f;


    throttle.begin(cfg);
}

// =====================
// CALIBRATION
// =====================


void captureMin() {
    throttleCal.minADC = analogRead(THROTTLE);
    Serial.print("Min captured: "); Serial.println(throttleCal.minADC);
}

void captureMax() {
    throttleCal.maxADC = analogRead(THROTTLE);
    Serial.print("Max captured: "); Serial.println(throttleCal.maxADC);
}

void applyCalibration() {

    if (throttleCal.maxADC <= throttleCal.minADC) {
        Serial.println("Invalid calibration");
        return;
    }

    THROTTLE_MIN_ADC = throttleCal.minADC + 5;
    THROTTLE_MAX_ADC = throttleCal.maxADC - 5;

    configureThrottle();

    Serial.println("Calibration applied");
}

// =====================
// DAC DEFAULTS
// =====================
int lastStoredDACStart = -1;

void updateDACDefaults() {
  return;
    if (DAC_START == lastStoredDACStart) return;

    // Set outputs first
    mcp.setChannelValue(MCP4728_CHANNEL_A, DAC_START);
    mcp.setChannelValue(MCP4728_CHANNEL_B, DAC_START);
  
    Serial.print("DAC_START ");
    Serial.println(DAC_START);

    mcp.setChannelValue(MCP4728_CHANNEL_C, 0);
    mcp.setChannelValue(MCP4728_CHANNEL_D, 0);

    mcp.saveToEEPROM();

    lastStoredDACStart = DAC_START;

    Serial.println("DAC defaults stored (Adafruit)");
}

// =====================
// TRACK OUTPUT
// =====================
void setTrackSpeed(int trackID, float speed) {

    speed = constrain(speed, 0.0f, 1.0f);

    int dac = DAC_START + speed * (DAC_MAX - DAC_START);
    dac = constrain(dac, 0, DAC_MAX);

    static int lastDAC[2] = {-1, -1};

   // if (dac != lastDAC[trackID]) {

        if (trackID == 0)
            mcp.setChannelValue(MCP4728_CHANNEL_A, dac);
        else
            mcp.setChannelValue(MCP4728_CHANNEL_B, dac);

        lastDAC[trackID] = dac;
        Serial.print("dac output:");
        Serial.println(dac);
   // }
}

// =====================
// THROTTLE MAP
// =====================
float mapThrottle(float t) {

    if (t < THROTTLE_DEADBAND)
        return 0.0f;

    if (t < TAKEUP_END) {
        float x = (t - THROTTLE_DEADBAND) / (TAKEUP_END - THROTTLE_DEADBAND);
        x = x * x;
        return x * 0.1f;
    }

    float x = (t - TAKEUP_END) / (1.0f - TAKEUP_END);
    x = pow(x, 1.3);

    float out = 0.1f + x * 0.9f;
    if (t > 0.95f) out = 1.0f;
    return out;

}

// =====================
// SERIAL
// =====================
void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
   
        if (c == '\n' || c == '\r') {
            Serial.print("Input string is ");
            Serial.println(inputString.c_str());
            processCommand(inputString);
            inputString = "";
        } else {

            inputString += c;
        }
    }
}

void printParams() {
    Serial.println("--- Params ---");
    Serial.print("DAC_START: "); Serial.println(DAC_START);
    Serial.print("Throttle Min: "); Serial.println(THROTTLE_MIN_ADC);
    Serial.print("Throttle Max: "); Serial.println(THROTTLE_MAX_ADC);
}

void processCommand(String cmd)
 {
  
    cmd.trim();
    Serial.println(cmd.c_str());

    if (cmd == "save") { saveConfig(); updateDACDefaults(); return; }
    if (cmd == "load") { if (!loadConfig()) loadDefaults(); configureThrottle(); return; }
    if (cmd == "defaults") { loadDefaults(); configureThrottle(); return; }

    if (cmd == "cal min") { captureMin(); return; }
    if (cmd == "cal max") { captureMax(); return; }
    if (cmd == "cal apply") { applyCalibration(); saveConfig(); return; }

    if (cmd == "show") { printParams(); return; }

    if (cmd.startsWith("start ")) 
    {

        DAC_START = cmd.substring(6).toInt();



        float volts = 4.7f * DAC_START / 4095.0f;

        Serial.print("DAC_START: ");
        Serial.print(DAC_START);

        Serial.print("  Voltage: ");
        Serial.print(volts, 2);
        Serial.println("V");

        return;
    }
}

// =====================
// SETUP
// =====================
void setup() {

    Serial.begin(115200);
    //Wire.begin();
    delay(3000);
    Serial.println("Running");
   
    if (!mcp.begin()) {
        Serial.println("MCP4728 not found!");
        while (1);

    }

    if (!loadConfig()) loadDefaults();
    Serial.println("Config loaded");

    configureThrottle();
    Serial.println("Throttle configured");
 
    updateDACDefaults();

    printParams();
}

// =====================
// LOOP
// =====================
int slowdown = 500;
void loop() {

    handleSerial();

    float throttleVal = throttle.GetThrottle();

    float target = mapThrottle(throttleVal);

    float rampRate = (target > currentOutput) ? RAMP_UP_RATE : RAMP_DOWN_RATE;
    currentOutput += (target - currentOutput) * rampRate;

    float leftSpeed  = currentOutput * (1.0f + trim);
    float rightSpeed = currentOutput * (1.0f - trim);

    setTrackSpeed(0, leftSpeed);
    setTrackSpeed(1, rightSpeed);
    if(slowdown)
    {
        slowdown--;
        if(slowdown == 0)
        {
            Serial.print("Track L: ");
            Serial.print(leftSpeed);
            Serial.print(" Track R: ");
            Serial.println(rightSpeed); 
            slowdown = 500;
        }
    }

}

