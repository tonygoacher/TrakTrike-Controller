/******************************************************************************
 *
 *  TrakTrike-Controller
 *
 *  An Arduino Nano based dual-track vehicle controller for tracked electric
 *  vehicles, skid-steer platforms and similar projects using analogue
 *  throttle controlled BLDC motor controllers.
 *
 *  Features:
 *      - Configurable throttle shaping
 *      - Multiple drive profiles
 *      - Track trim calibration and interpolation
 *      - Dual MCP4728 DAC outputs
 *      - EEPROM configuration storage
 *      - Throttle calibration and diagnostics
 *
 *  Author:
 *      Tony Goacher
 *
 *  Project:
  *     https://github.com/tonygoacher/TrakTrike-Controller
 *
 *  Copyright:
 *      Copyright (c) 2026 Tony Goacher
 *
 *  License:
 *      Released under the MIT License.
 *
 *  Disclaimer:
 *      This software controls equipment capable of movement and may cause
 *      injury, death, or property damage if used incorrectly.
 *
 *      The software and associated hardware design is provided 
 *      "as is", without warranty of any kind,
 *      express or implied. The author accepts no liability for any loss,
 *      damage, injury, or consequences arising from its use.
 *
 *      Users are solely responsible for ensuring that the software and hardware is
 *      suitable for their application and that appropriate safety measures
 *      are implemented and tested before use.
 *
 *      Always test with drive wheels or tracks lifted clear of the ground
 *      before operating a vehicle under software control.
 *
 ******************************************************************************/

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <stddef.h>


#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <Adafruit_MCP4728.h>
#include "ports.h"

#include "Switch.h"
#include "SmoothBarGraph.h"
#include "Pacer.h"



// =====================
// MCP4728
// =====================
Adafruit_MCP4728 mcp;
constexpr int   MinDACValue = 0;
constexpr int   MaxDACValue = 4095;
constexpr float DAC_VREF    = 4.7f;

// =====================
// CONFIG
// =====================
#define EEPROM_ADDR 0
#define PARAM_VERSION 7
#define NUM_TRIM_VALUES 11

#define STATS_UPDATE_MS 1000    // Update stats this many ms
#define BARGRAPH_UPDATE_MS 100  // Update bargraph this many ms

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
SmoothBarGraph barGraph(lcd,6,1,10);

struct DriveProfile
{
    float curveExponent;
    float rampUp;
    float rampDown;
};




DriveProfile normalProfile =
{
    1.3f,   // curve
    0.05f,  // ramp up
    0.8f    // ramp down
};

DriveProfile slowProfile =
{
    1.5f,   // softer curve
    0.005f,  // gentler ramp
    0.15f    // Ramp down
};

DriveProfile brakeProfile =
{
    0.0f,   // softer curve
    0.0f,  // gentler ramp
    1.0f    // HArd decel
};


// Use force command to set these values Then measure both motor speeds.
// The use calibrate lefftrpm rightrpm to set the TRIM_VALUE entry. Index into TRIM_VALUE is calculated automatically
const float trimThrottlePoints[NUM_TRIM_VALUES] =
{
    0.0f,
    0.1f,
    0.2f,
    0.3f,
    0.4f,
    0.5f,
    0.6f,
    0.7f,
    0.8f,
    0.9f,
    1.0f
};

// This can be used to override the twist throttle value during trim calibration. BRAKEMODE overrides this.
float FORCE_OUTPUT  = 0.0f;

// Default trim calibration values. These are for my EV. 
// You probably need to do the calibration routine to find your defaults
const float defaultCalibration[NUM_TRIM_VALUES] = 
{
    0.17,
    -0.16,
    0.03,
    0.08,
    0.12,
    0.19,
    0.16,
    0.10,
    0.17,
    0.17,
    0.17,
};

enum TRACK_ID
{
    LEFT = 0,
    RIGHT = 1
};

enum SystemMode
{
    SLOWMODE = 1,
    BRAKEMODE = 2,
    REVERSEMODE = 4,
    FORCEMODE = 8,
    MODECHANGE = 16,
 

    INIT = 0xff
};

// Pending mode state being assembled by SetModes().
// Changes are committed to systemMode by
// displayNewSystemMode() after one-shot actions
// (LCD updates, output resets, etc.) have been performed.
uint8_t newSystemMode = SystemMode::SLOWMODE; 

// Current active mode state.
// Used to detect mode transitions and drive the LCD display.
uint8_t currentSystemMode = SystemMode::INIT;  

 // When changing mode, force the throttle to zero for 500ms or continuously until throttle released
// This prevents unwanted movement if reverse is enabled at none aero throttle
Pacer modePacer(false,500);   

typedef struct {
    uint8_t  version;
    uint16_t crc;
} config_header_t;

struct settings_t
{
    int LEFT_DAC_START;
    int LEFT_DAC_MAX;
    int LEFT_SLOW_DAC_MAX;
    int RIGHT_DAC_START;
    int RIGHT_DAC_MAX;
    int RIGHT_SLOW_DAC_MAX;
    int LEFT_DEFAULT;
    int RIGHT_DEFAULT;

    float THROTTLE_DEADBAND;
    float TAKEUP_END;

    float TRIM_VALUES[NUM_TRIM_VALUES];


    int THROTTLE_MIN_ADC;
    int THROTTLE_MAX_ADC;
    uint8_t DAC_DEFAULT_WRITTEN;       // This is used to track if the default DAC output has been written to the DAC EEPROM
};

struct Config {
    config_header_t header;
    settings_t  settings;
   
};

Config cfg;



// Declare forward refs
void  loadDefaults();


void printTrimValues( const float v[NUM_TRIM_VALUES])
{
    for(int i = 0 ; i < NUM_TRIM_VALUES ; i++)
    {   
       Serial.println(v[i]);
    }
}

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

String inputString = "";
Switch modeSwitch(MODE);


// =====================
// FORWARD DECLARATIONS
// =====================
void processCommand(String cmd);
void updateDACDefaults();
void applyCalibration();
void configureThrottle();
void printParams();

void printAll(const Config& cfg)
{
    Serial.println(F("===== CONFIG ====="));

    Serial.print(F("Version: "));
    Serial.println(cfg.header.version);

    Serial.print(F("CRC: "));
    Serial.println(cfg.header.crc);

    Serial.println();

    Serial.print(F("LEFT DAC_START: "));
    Serial.println(cfg.settings.LEFT_DAC_START);
    Serial.print(F("RIGHT DAC_START: "));
    Serial.println(cfg.settings.RIGHT_DAC_START);    

    Serial.print(F("LEFT_DAC_MAX: "));
    Serial.println(cfg.settings.LEFT_DAC_MAX);
    
    Serial.print(F("LEFT_SLOW_DAC_MAX: "));
    Serial.println(cfg.settings.LEFT_SLOW_DAC_MAX);

    Serial.print(F("RIGHT DAC_MAX: "));
    Serial.println(cfg.settings.RIGHT_DAC_MAX);

    Serial.print(F("RIGHT_SLOW_DAC_MAX: "));
    Serial.println(cfg.settings.RIGHT_SLOW_DAC_MAX);

    Serial.println();

    Serial.print(F("THROTTLE_DEADBAND: "));
    Serial.println(cfg.settings.THROTTLE_DEADBAND, 4);

    Serial.print(F("TAKEUP_END: "));
    Serial.println(cfg.settings.TAKEUP_END, 4);

    Serial.println();

    Serial.println();

    Serial.println(F("Config trim values"));
    printTrimValues( cfg.settings.TRIM_VALUES);
    

    Serial.print(F("THROTTLE_MIN_ADC: "));
    Serial.println(cfg.settings.THROTTLE_MIN_ADC);

    Serial.print(F("THROTTLE_MAX_ADC: "));
    Serial.println(cfg.settings.THROTTLE_MAX_ADC);

    Serial.println(F("=================="));
}

// =====================
// CRC
// =====================
uint16_t calculateCRC(const settings_t &cfg) {
    const uint8_t *data = (const uint8_t*)&cfg;

    size_t len   = sizeof(settings_t) ;

    uint16_t crc = 0;

    for (size_t i = 0; i < len; i++) {
        crc += data[i];
    }

    return crc;
}

// =====================
// CONFIG APPLY
// =====================
void applyConfig(const Config &new_cfg) {

    
    cfg = new_cfg;
    printAll(cfg);
}

// =====================
// CONFIG SAVE/LOAD
// =====================
void saveConfig() {
  

    cfg.header.version = PARAM_VERSION;


    cfg.header.crc = calculateCRC(cfg.settings);

    EEPROM.put(EEPROM_ADDR, cfg);

    Serial.println("Saved config");
}

bool loadConfig() {
    Config loadedConfig;
    EEPROM.get(EEPROM_ADDR, loadedConfig);

    if (loadedConfig.header.version != PARAM_VERSION) 
    {
        Serial.println("Config version mismatch. Using defaults");
        loadDefaults();
        saveConfig();
        return false;
    }

    if (calculateCRC(loadedConfig.settings) != loadedConfig.header.crc)
    {
      Serial.println("Config load failed CRC. Using defaults");
      loadDefaults();
      saveConfig();
      return false;
    }      

    applyConfig(loadedConfig);

    Serial.println("Loaded config");
    return true;
}

void loadDefaults() {

     cfg.header.version = PARAM_VERSION;
     cfg.header.crc = 0;    // This will be set in the save routine.
     cfg.settings.LEFT_DAC_START = 1060;
     cfg.settings.RIGHT_DAC_START = 1060;
     cfg.settings.LEFT_DAC_MAX = 3500;
     cfg.settings.RIGHT_DAC_MAX = 3500;
     cfg.settings.LEFT_SLOW_DAC_MAX = 1500;
    
     cfg.settings.RIGHT_SLOW_DAC_MAX = 1500;

     cfg.settings.THROTTLE_DEADBAND = 0.05f;
     cfg.settings.TAKEUP_END = 0.20f;


    for(int i = 0 ; i < NUM_TRIM_VALUES ; i++)
    {
         cfg.settings.TRIM_VALUES[i] = defaultCalibration[i];
    }


     cfg.settings.THROTTLE_MIN_ADC = 177;
     cfg.settings.THROTTLE_MAX_ADC = 808;

     cfg.settings.DAC_DEFAULT_WRITTEN = 0;

    Serial.println("Loaded defaults");
    printParams();
    
}

// =====================
// THROTTLE SETUP
// =====================
void configureThrottle() {

    TwistThrottle::Config throttleConfig;

    throttleConfig.minADC =  cfg.settings.THROTTLE_MIN_ADC;
    throttleConfig.maxADC =  cfg.settings.THROTTLE_MAX_ADC;

    throttleConfig.deadband = 8;
    throttleConfig.filterAlpha = 0.1f;
    throttleConfig.maxRate = 0.0f;


    throttle.begin(throttleConfig);
}

// =====================
// CALIBRATION
// =====================


void captureMin() {
    throttleCal.minADC = analogRead(THROTTLE);
    Serial.print(F("Min captured: ")); Serial.println(throttleCal.minADC);
}

void captureMax() {
    throttleCal.maxADC = analogRead(THROTTLE);
    Serial.print(F("Max captured: ")); Serial.println(throttleCal.maxADC);
}

/**
 * Apply the most recently captured throttle calibration values.
 *
 * The calibration process records the minimum and maximum ADC readings
 * observed while the throttle is moved through its full travel. This
 * function validates the captured range, applies a small safety margin,
 * and updates the stored throttle calibration limits.
 *
 * A margin of 5 ADC counts is added to the measured minimum and
 * subtracted from the measured maximum. This prevents normal operation
 * from occurring exactly at the calibration boundaries and provides a
 * small tolerance for ADC noise and component drift.
 *
 * The adjusted values are constrained to the valid ADC range
 * (0-1023) and then checked to ensure a valid throttle span remains.
 *
 * If the resulting range is invalid, the calibration is rejected and
 * detailed diagnostic information is printed to aid troubleshooting.
 *
 * On success, configureThrottle() is called to rebuild any derived
 * parameters used by the throttle processing code.
 */
void applyCalibration()
{
    constexpr int MaxADCValue = 1023;  // This may change depending on yuor hardware
    constexpr int MinADCValue = 0;
    constexpr int CALIBRATION_MARGIN_ADC = 5;   // Gives us some leeway over a noise ADC reading

    // Validate measured calibration range
    if (throttleCal.maxADC <= throttleCal.minADC)
    {
        Serial.println(F("Invalid calibration range"));

        Serial.print(F("Measured Min: "));
        Serial.println(throttleCal.minADC);

        Serial.print(F("Measured Max: "));
        Serial.println(throttleCal.maxADC);

        return;
    }

    // Apply calibration margin and constrain to ADC limits
    int minADC = constrain(throttleCal.minADC + CALIBRATION_MARGIN_ADC, MinADCValue, MaxADCValue);
    int maxADC = constrain(throttleCal.maxADC - CALIBRATION_MARGIN_ADC, MinADCValue, MaxADCValue);

    // Ensure a valid range remains after margins are applied
    if (maxADC <= minADC)
    {
        Serial.println(F("Calibration range too small after margins"));

        Serial.print(F("Measured Min: "));
        Serial.println(throttleCal.minADC);

        Serial.print(F("Measured Max: "));
        Serial.println(throttleCal.maxADC);

        Serial.print(F("Adjusted Min: "));
        Serial.println(minADC);

        Serial.print(F("Adjusted Max: "));
        Serial.println(maxADC);

        return;
    }

    cfg.settings.THROTTLE_MIN_ADC = minADC;
    cfg.settings.THROTTLE_MAX_ADC = maxADC;

    configureThrottle();

    Serial.println(F("Calibration applied"));

    Serial.print(F("Throttle Min ADC: "));
    Serial.println(cfg.settings.THROTTLE_MIN_ADC);

    Serial.print(F("Throttle Max ADC: "));
    Serial.println(cfg.settings.THROTTLE_MAX_ADC);
}


void updateDACDefaults() {


    Serial.println(F("Updating DAC defaults"));

    // Store a power-up default of 0V in the DAC EEPROM.
    // Some motor controllers refuse to enable if any throttle
    // voltage is present during their startup sequence.
    mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
    mcp.setChannelValue(MCP4728_CHANNEL_B, 0);
  
    mcp.setChannelValue(MCP4728_CHANNEL_C, 0);
    mcp.setChannelValue(MCP4728_CHANNEL_D, 0);

    mcp.saveToEEPROM();

}


// The throttle value is always 0.0-1.0. This function returns the maximum DAC value 
// that is to be scaled afgainst the throttle value to get the actual speed. SLOWMODE returns a
// smaller MAXDAC value so we can use the entire mechanical range of the twist throttle but 
// limit the maximum speed that this will result in.
int getMaxDACValue(int trackID)
{
    // Force mode overrides all as it's essential we use the top end DAC output for calibration
    if(((currentSystemMode & SystemMode::SLOWMODE) || (currentSystemMode & SystemMode::REVERSEMODE)) && !(currentSystemMode & SystemMode::FORCEMODE))
    {
        if(trackID == TRACK_ID::LEFT)
        {
            return cfg.settings.LEFT_SLOW_DAC_MAX;
        }
        else
        {
            return cfg.settings.RIGHT_SLOW_DAC_MAX;
        }
    }
    
    return trackID == TRACK_ID::LEFT ? cfg.settings.LEFT_DAC_MAX : cfg.settings.RIGHT_DAC_MAX;

}


// =====================
// TRACK OUTPUT
// =====================
Pacer trackPacer(true,STATS_UPDATE_MS);
void setTrackSpeed(int trackID, float speed, bool forceWrite)
{

    speed = constrain(speed, 0.0f, 1.0f);


    int vmax = getMaxDACValue(trackID); // Ge the max value to scale againt the throttle

    int vstart = (trackID == TRACK_ID::LEFT)    // Any values lower than vstart result in the motor being off, so start at this value
             ? cfg.settings.LEFT_DAC_START
             : cfg.settings.RIGHT_DAC_START;

    int dac = vstart + speed * (vmax - vstart);
    dac = constrain(dac, 0, vmax);

    static int lastDAC[2] = {-1, -1};



    if (dac != lastDAC[trackID] || forceWrite) {

        if (trackID == 0)
            mcp.setChannelValue(MCP4728_CHANNEL_A, dac);
        else
            mcp.setChannelValue(MCP4728_CHANNEL_B, dac);

        lastDAC[trackID] = dac;
    }
    
    if(trackPacer.Pace())
    {
        Serial.print(F("Left DAC: ")); Serial.print(lastDAC[0]); Serial.print(F("  Right DAC: ")); Serial.println(lastDAC[1]);
    }
}

// =====================
// THROTTLE MAP
/**
 * Convert a normalised throttle input (0.0-1.0) into a normalised
 * speed demand (0.0-1.0).
 *
 * The mapping consists of three regions:
 *
 *  1) Deadband
 *     Inputs below THROTTLE_DEADBAND return zero output. This removes
 *     sensor noise and prevents unintended vehicle movement.
 *
 *  2) Take-up region
 *     Between THROTTLE_DEADBAND and TAKEUP_END a quadratic curve is
 *     applied, producing an output range of 0.0-0.1. This provides
 *     fine low-speed control and reduces the tendency for the vehicle
 *     to "jump" as movement begins.
 *
 *  3) Main throttle region
 *     Above TAKEUP_END the remaining throttle travel is scaled to
 *     produce an output range of 0.1-1.0. The DriveProfile
 *     curveExponent controls throttle feel:
 *
 *         1.0 = linear
 *        >1.0 = softer response
 *        <1.0 = more aggressive response
 *
 * A small full-throttle override ensures 100% output can always be
 * reached even if the calibrated throttle never quite reaches 1.0.
 *
 * @param t        Normalised throttle input (0.0-1.0)
 * @param profile  Active drive profile defining throttle shaping
 *
 * @return Normalised speed demand (0.0-1.0)
 */
float mapThrottle(float t, const DriveProfile& profile)
{

    if (t < cfg.settings.THROTTLE_DEADBAND)
        return 0.0f;

    if (t < cfg.settings.TAKEUP_END) {
        float x = (t - cfg.settings.THROTTLE_DEADBAND) / (cfg.settings.TAKEUP_END - cfg.settings.THROTTLE_DEADBAND);
        x = x * x;
        return x * 0.1f;
    }

    float x = (t - cfg.settings.TAKEUP_END) / (1.0f - cfg.settings.TAKEUP_END);

    x = pow(x, profile.curveExponent);


    float out = 0.1f + x * 0.9f;

    if (t > 0.95f)
        return 1.0f;

    return out;

}



// =====================
// We use the serial port during calibration.
// =====================
void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
   
        if (c == '\n' || c == '\r') {
            Serial.print(F("Input string is "));
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
    Serial.print(F("LEFT_DAC_START: ")); Serial.println(cfg.settings.LEFT_DAC_START);
    Serial.print(F("RIGHT_DAC_START: ")); Serial.println(cfg.settings.RIGHT_DAC_START);
    Serial.print(F("Throttle Min: ")); Serial.println(cfg.settings.THROTTLE_MIN_ADC);
    Serial.print(F("Throttle Max: ")); Serial.println(cfg.settings.THROTTLE_MAX_ADC);
    Serial.println(F("Trims: Point    Value"));
    for(int i = 0 ; i < NUM_TRIM_VALUES ; i++)
    {
        Serial.print(F("      "));Serial.print(trimThrottlePoints[i]); Serial.print("      "); Serial.println(cfg.settings.TRIM_VALUES[i]);
    }
}

void setDACMax(TRACK_ID track, int value)
{

    value = constrain(value, MinDACValue, MaxDACValue);

    if(track == LEFT)
    {
        cfg.settings.LEFT_DAC_MAX = value;
        Serial.print(F("LEFT DAC MAX"));
    }
    else
    {
        cfg.settings.RIGHT_DAC_MAX = value;
        Serial.print(F("RIGHT DAC MAX"));
    }

    float volts = DAC_VREF * value / MaxDACValue;;

    Serial.print(F(": "));
    Serial.print(value);

    Serial.print(F("  Voltage: "));
    Serial.print(volts, 2);
    Serial.println(F("V"));
}

void setSlowDACMax(TRACK_ID track, int value)
{
    constexpr int   MinDACValue = 0;
    constexpr int   MaxDACValue = 4095;
    constexpr float DAC_VREF    = 4.7f;

    value = constrain(value, MinDACValue, MaxDACValue);

    if(track == LEFT)
    {
        cfg.settings.LEFT_SLOW_DAC_MAX = value;
        Serial.print(F("LEFT SLOW DAC MAX"));
    }
    else
    {
        cfg.settings.RIGHT_SLOW_DAC_MAX = value;
        Serial.print(F("RIGHT SLOW DAC MAX"));
    }

    float volts = DAC_VREF * value / MaxDACValue;

    Serial.print(F(": "));
    Serial.print(value);

    Serial.print(F("  Voltage: "));
    Serial.print(volts, 2);
    Serial.println(F("V"));
}


/**
 * Set the minimum DAC value for the specified track.
 *
 * DAC_START defines the output value applied when the commanded track
 * speed is zero. This is used to compensate for motor controller deadband,
 * ensuring the tracks begin moving as soon as a non-zero speed is requested.
 * NB It is not the absolute minimum value as a value of 0 is forced for 2s during start up
 *
 * The function updates the configuration value and reports the equivalent
 * output voltage for convenience when tuning the controller. The calculated
 * voltage assumes a 4.7V MCP4728 reference supply and a 12-bit DAC range
 * of 0-4095 counts.
 *
 * @param track       Track to update (LEFT or RIGHT)
 * @param startValue  DAC value in the range 0-4095
 */
void setDACStart(TRACK_ID track, int startValue)
{
    if(track == LEFT)
    {
        cfg.settings.LEFT_DAC_START = startValue;  
        Serial.print(F("LEFT DAC START"));
    }
    else
    {
        cfg.settings.RIGHT_DAC_START = startValue;
        Serial.print(F("RIGHT DAC START"));
    }

    float volts = 4.7f * startValue / 4095.0f;

    Serial.print(" :");
    Serial.print(startValue);

    Serial.print(F("  Voltage: "));
    Serial.print(volts, 2);
    Serial.println("V");
}



/**
 * Calculate and store a trim correction for the current throttle position.
 *
 * The TrakTrike uses a table of trim values indexed by throttle position to
 * compensate for differences between the left and right drive systems.
 *
 * During calibration the vehicle is driven at a known throttle setting (Entries in trimThrottlePoints) and
 * the actual left and right track RPMs are measured with a tachometer. The nearest throttle
 * calibration point is located and a signed trim value is calculated from
 * the RPM difference:
 *
 *   Positive trim -> reduce right track speed
 *   Negative trim -> reduce left track speed
 *
 * To prevent accidental calibration at arbitrary throttle positions, the
 * current throttle must be within ±0.02 of a predefined trim table point.
 *
 * The resulting trim value is stored in cfg.settings.TRIM_VALUES[] and is
 * later interpolated by getInterpolatedTrim() to provide smooth correction
 * across the entire throttle range.
 *
 * @param throttle  Current commanded throttle (0.0-1.0)
 * @param leftRPM   Measured left track RPM
 * @param rightRPM  Measured right track RPM
 */
void calibrate(float throttle,
               float leftRPM,
               float rightRPM)
{
    Serial.println();
    Serial.println(F("=== CALIBRATION ==="));

    Serial.print(F("Throttle: "));
    Serial.println(throttle, 3);

    Serial.print(F("Left RPM: "));
    Serial.println(leftRPM, 1);

    Serial.print(F("Right RPM: "));
    Serial.println(rightRPM, 1);

    // Sanity check
    if (leftRPM < 1.0f || rightRPM < 1.0f)
    {
        Serial.println(F("Calibration aborted: RPM too low"));
        return;
    }

    // Find nearest trim table point
    int bestIndex = -1;
    float bestError = 999.0f;

    for (int i = 0; i < NUM_TRIM_VALUES; i++)
    {
        float error = abs(throttle - trimThrottlePoints[i]);

        Serial.print(F("i="));
        Serial.print(i);

        Serial.print(F(" point="));
        Serial.print(trimThrottlePoints[i], 3);

        Serial.print(F(" error="));
        Serial.println(error, 3);

        if (error < bestError)
        {
            bestError = error;
            bestIndex = i;
        }
    }

    // Safety check
    if (bestIndex < 0)
    {
        Serial.println(F("Calibration aborted: no valid index"));
        return;
    }

    // Require throttle to be reasonably close
    if (bestError > 0.02f)
    {
        Serial.println(F("Calibration aborted: throttle not near trim point"));

        Serial.print(F("Nearest point: "));
        Serial.println(trimThrottlePoints[bestIndex], 3);

        return;
    }

    // Calculate signed trim
    //
    // Positive trim  -> slow RIGHT track
    // Negative trim  -> slow LEFT track
    //

    float trim = 0.0f;

    if (rightRPM > leftRPM)
    {
        trim = 1.0f - (leftRPM / rightRPM);
    }
    else
    {
        trim = -(1.0f - (rightRPM / leftRPM));
    }

    // Store trim
    cfg.settings.TRIM_VALUES[bestIndex] = trim;

    // Report result
    Serial.println();
    Serial.print(F("Stored trim at index "));
    Serial.println(bestIndex);

    Serial.print(F("Trim point: "));
    Serial.println(trimThrottlePoints[bestIndex], 3);

    Serial.print(F("Trim value: "));
    Serial.println(trim, 4);

    Serial.println(F("==================="));
    Serial.println();
}


// Act upon a serial command  for calibration
void processCommand(String cmd)
 {
  
    cmd.trim();
    Serial.println(cmd.c_str());

    if (cmd == F("save")) { saveConfig(); updateDACDefaults(); return; }
    if (cmd == F("load")) { loadConfig(); configureThrottle(); return; }
    if (cmd == F("defaults")) { loadDefaults(); configureThrottle(); return; }

    if (cmd == F("cal min")) { captureMin(); return; }
    if (cmd == F("cal max")) { captureMax(); return; }
    if (cmd == F("cal apply")) { applyCalibration(); saveConfig(); return; }

    if (cmd == F("show")) { printParams(); return; }

    if (cmd == F("showall")) { printAll(cfg); return; }

    if(cmd.startsWith(F("force ")))
    {
        FORCE_OUTPUT = cmd.substring(6).toFloat();
        return;
    }

    if (cmd.startsWith(F("startl "))) 
    {
          setDACStart(TRACK_ID::LEFT,cmd.substring(6).toInt());
        return;
    }

    if (cmd.startsWith(F("startr "))) 
    {
        setDACStart(TRACK_ID::RIGHT,cmd.substring(6).toInt());

        return;
    }
    //                     111111111
    //           0123456789012345678          
    // Format is calibrate llll rrrr
    if (cmd.startsWith(F("calibrate"))) 
    {
        if(cmd == F("calibrate"))
        {
            Serial.println("Trim throttle points:");
            printTrimValues(trimThrottlePoints);
            Serial.println(F("Current Trim Values"));
            printTrimValues( cfg.settings.TRIM_VALUES);

            return;
        }

        int leftSpeed = cmd.substring(10).toInt();
        int indexRight = cmd.indexOf(' ', 10);  // Get index of next space
        int rightSpeed = cmd.substring(indexRight+1).toInt();

        Serial.print("Cal Left: "); Serial.print(leftSpeed); Serial.print("  Right: ");Serial.println(rightSpeed);
        
        float commandedThrottle;
        if (FORCE_OUTPUT)
        {
            commandedThrottle = FORCE_OUTPUT;
        }
        else
        {
            commandedThrottle = throttle.GetThrottle();
        }
        calibrate(commandedThrottle, leftSpeed,rightSpeed);
        return;
    }

    if (cmd.startsWith("dacmaxl "))
    {
        setDACMax(LEFT, cmd.substring(8).toInt());
        return;
    }

    if (cmd.startsWith("dacmaxr "))
    {
        setDACMax(RIGHT, cmd.substring(8).toInt());
        return;
    }

    if (cmd.startsWith("slowmaxl "))
    {
        setSlowDACMax(LEFT, cmd.substring(9).toInt());
        return;
    }

    if (cmd.startsWith("slowmaxr "))
    {
        setSlowDACMax(RIGHT, cmd.substring(9).toInt());
        return;
    }

    Serial.print(F("UNKNOWN COMMAND ")); Serial.println(cmd.c_str());
}

// =====================
// SETUP
// =====================
void setup() {

    Serial.begin(115200);

  
    Serial.println(F("Running"));

    lcd.init();
    lcd.backlight();
    lcd.print(F(" TrakTrike v4.0 "));
    lcd.setCursor(0,1);
    lcd.print(F("Initialising..."));

    pinMode(BRAKE,INPUT_PULLUP);
    pinMode(REVERSE,INPUT_PULLUP);
      
    if (!mcp.begin()) {
        Serial.println(F("MCP4728 not found!"));
        lcd.setCursor(0,1);
        lcd.print(F("MCP4728 failed!"));
        while (1);
    }

    // It was found that ANY voltage output on the throttle input to the motor controller
    // at power up can cause the controller to ignore any further throttle values.
    // We force the throttle here for 3s to allow it to settle before we continue
    mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
    mcp.setChannelValue(MCP4728_CHANNEL_B, 0);



    delay(3000);

    loadConfig();
  

    configureThrottle();
   

    delay(500);

    printParams();

    if(cfg.settings.DAC_DEFAULT_WRITTEN == 0)
    {
         updateDACDefaults();
         cfg.settings.DAC_DEFAULT_WRITTEN = 0xff;
         saveConfig();
    }
    else
    {
        Serial.println(F("DAC defaults aleady set"));
    }
    Serial.println(F("System OK"));

    barGraph.begin();
}

/**
 * Evaluate all operating modes and select the drive profile for the
 * current control loop.
 *
 * Mode bits are updated on every call to ensure newSystemMode always
 * reflects the current state of the vehicle inputs, regardless of
 * profile priority.
 *
 * Profiles are selected according to the following priority:
 *
 *   BRAKE    - Highest priority. Applies brakeProfile.
 *   FORCE    - Calibration mode. Uses normalProfile to provide
 *              the full DAC output range.
 *   REVERSE  - Uses slowProfile for controlled reversing.
 *   SLOW     - Operator-selected manoeuvring mode.
 *   NORMAL   - Default operating mode.
 *
 * Once a profile has been selected by a higher-priority mode, lower
 * priority modes may still update their mode bits but cannot replace
 * the selected profile.
 *
 * The returned profile is guaranteed to be valid. If an unexpected
 * condition leaves no profile selected, brakeProfile is used as a
 * fail-safe.
 * 
 * Note that newSystemMode is the value the mode needs to be. It may already be in
 * the correct state from a previous iteration. It will be copied to currentSystemState 
 * if it's value is different
 *  
 *
 * @return Pointer to the DriveProfile for this control loop.
 */
DriveProfile* SetModes()
{
    DriveProfile* profile = NULL;   // Only update the profile if it has not already been set


    if(digitalRead(BRAKE) == LOW)
    {
        if(!(currentSystemMode & SystemMode::BRAKEMODE))
        {
            modePacer.PacerReset();
        }  
        newSystemMode |= SystemMode::BRAKEMODE; 
        profile = &brakeProfile;    // This overrides everything
    }
    else
    {
        if((currentSystemMode & SystemMode::BRAKEMODE))
        {
            modePacer.PacerReset();
        } 
        newSystemMode &= ~SystemMode::BRAKEMODE;    
    }

    if(FORCE_OUTPUT > 0.0)
    {
        newSystemMode |= SystemMode::FORCEMODE;
        // Only select this profile if a higher-priority mode
        // has not already selected one.
        if(profile  == NULL)    
        {
            profile = &normalProfile;   // FORCE_MODE requires full DAC range so force it here
        }            
    }
    else
    {
        newSystemMode &=~ SystemMode::FORCEMODE;
    }

    if(digitalRead(REVERSE) == LOW)
    {
        if(!(currentSystemMode & SystemMode::REVERSEMODE))
        {
            modePacer.PacerReset();
        }           

        newSystemMode |= SystemMode::REVERSEMODE;
        if(profile == NULL)
        {
            profile =   &slowProfile;  
        }
    
    }
    else
    {
        if((currentSystemMode & SystemMode::REVERSEMODE))
        {
            modePacer.PacerReset();
        }    
        newSystemMode &= ~SystemMode::REVERSEMODE;
    }

    if(newSystemMode & SystemMode::SLOWMODE)
    {

        if(!(currentSystemMode & SystemMode::SLOWMODE))
        {
            modePacer.PacerReset();
        }  

        if(profile == NULL)
        {
            profile =  &slowProfile;  
        }
     
    }
    else
    {
        if(currentSystemMode & SystemMode::SLOWMODE)
        {
            modePacer.PacerReset();
        }  
        if(profile == NULL)
        {
           profile = &normalProfile;
        }            
    }
   
    if(profile == NULL)
    {
        Serial.print(F("ERROR : MODE IS UNDEFINED! : ")); Serial.print(currentSystemMode);
        profile = &brakeProfile;
    }

    if(modePacer.Running())
    {
        newSystemMode |= SystemMode::MODECHANGE;  
    }
    else
    {
        newSystemMode &= ~SystemMode::MODECHANGE;  
    }
    return profile;
}


/**
 * Process pending mode changes and update the LCD display.
 *
 * newSystemMode contains the mode state assembled by SetModes().
 * This function compares it against currentSystemMode and performs
 * any one-shot actions required when the mode changes.
 *
 * When a change is detected:
 *   - currentSystemMode is updated
 *   - the mode indicator on the LCD is refreshed
 *   - a boolean result indicates whether the drive output should
 *     be reset to zero
 *
 * Return value:
 *   true  - drive output should be reset (BRAKE, REVERSE, SLOW)
 *   false - no output reset required (FORCE, NORMAL, or no change)
 *
 * This mechanism prevents repeated LCD updates and provides a single
 * point for handling mode transition events.
 *
 * @return true if the caller should reset currentOutput, otherwise false.
 */
bool displayNewSystemMode()
{

    if(newSystemMode != currentSystemMode)
    {

        currentSystemMode = newSystemMode;
        lcd.setCursor(0,1);

        if(currentSystemMode & SystemMode::MODECHANGE)
        {
            lcd.print(F("T<>0 "));
            return true;
        }  

        if(currentSystemMode & SystemMode::BRAKEMODE)
        {
            lcd.print(F("BRKE "));
            return true;
        }

        if(currentSystemMode & SystemMode::FORCEMODE)
        {
            lcd.print(F("FRCE  "));
            return false;
        }

        if(currentSystemMode & SystemMode::REVERSEMODE)
        {
            lcd.print(F("RVRS "));
            return true;
        }

        if(currentSystemMode & SystemMode::SLOWMODE)
        {
            lcd.print(F("SLOW "));
            return true;
        }    
        

        lcd.print(F("NORM "));   

    }
    return false;
}


/**
 * Trim table support functions.
 *
 * Track trim is stored as a set of calibration points distributed across
 * the throttle range. Each entry in TRIM_VALUES[] contains the correction
 * required at the corresponding throttle point in trimThrottlePoints[].
 *
 * getTrimIndex()
 *   Converts a normalised throttle value (0.0-1.0) into the lower trim
 *   table index. This is primarily used during calibration when storing
 *   a trim value at the nearest throttle calibration point.
 *
 * getInterpolatedTrim()
 *   Returns the trim correction for an arbitrary throttle value by
 *   linearly interpolating between adjacent entries in the trim table.
 *   This produces a smooth trim curve and avoids abrupt steering changes
 *   when transitioning between calibration points.
 *
 * Example:
 *
 *   Throttle Points:  0.0   0.1   0.2   0.3
 *   Trim Values:     0.00  0.01  0.03  0.04
 *
 *   throttle = 0.15
 *
 *   Interpolated trim = 0.02
 *
 * The trim value is later applied by the drive controller to compensate
 * for differences between the left and right drive systems.
 */
int getTrimIndex(float throttle)
{
    float scaled = throttle * (NUM_TRIM_VALUES - 1);

    // Lower table index
    int index = (int)scaled;
    return index;
}


float getInterpolatedTrim(float throttle)
{
    // Clamp input
    if (throttle < trimThrottlePoints[0])
        return cfg.settings.TRIM_VALUES[0];

    if (throttle >= trimThrottlePoints[NUM_TRIM_VALUES - 1])
        return cfg.settings.TRIM_VALUES[NUM_TRIM_VALUES - 1];

    // Find containing segment
    for (int i = 0; i < (NUM_TRIM_VALUES - 1); i++)
    {
        float t0 = trimThrottlePoints[i];
        float t1 = trimThrottlePoints[i + 1];

        if (throttle >= t0 && throttle <= t1)
        {
            float frac = (throttle - t0) / (t1 - t0);

            return cfg.settings.TRIM_VALUES[i] +
                   (cfg.settings.TRIM_VALUES[i + 1] - cfg.settings.TRIM_VALUES[i]) * frac;
        }
    }

    // Should never happen
    return 0.0f;
}

/*
* Display the current mode on the serial interface
*/
void showMode()
{
    Serial.print(F("MODE: "));
    if(currentSystemMode & SystemMode::SLOWMODE)
    {
        Serial.print(F(" SLOW"));
    }
    else
    {
        Serial.print(F(" NORMAL"));
    
    }
       
    if(currentSystemMode & SystemMode::BRAKEMODE)
    {
        Serial.print(F(" BRAKE"));
    }

    if(currentSystemMode & SystemMode::FORCEMODE)
    {
        Serial.print(F(" FORCE"));
    }

    if(currentSystemMode & SystemMode::REVERSEMODE)
    {
        Serial.print(F(" REVERSE"));
    }

    if(currentSystemMode & SystemMode::MODECHANGE)
    {
        Serial.print(F(" T<>0"));
    }
    Serial.println();

}

// =====================
// LOOP
// =====================
float currentOutput = 0.0f;
Pacer lcdPacer(true,BARGRAPH_UPDATE_MS);
Pacer serialPacer(true,STATS_UPDATE_MS);

void loop() 
{   
    static float lastValue = -1.0f;
    handleSerial();

    float throttleVal = throttle.GetThrottle();

    DriveProfile* profile = SetModes();

    // Force a controlled re-ramp whenever the drive mode changes.
    // This prevents abrupt speed changes when transitioning between
    // normal, slow, reverse and brake profiles.
    if(displayNewSystemMode())
    {
        currentOutput = 0.0f;
    }

    float target = mapThrottle(throttleVal, *profile);

    float rampRate = (target > currentOutput) ?
                 profile->rampUp :
                 profile->rampDown;

    currentOutput += (target - currentOutput) * rampRate;

 
    // Trim is applied based on the final commanded output rather than
    // raw throttle position so that interpolation follows the actual
    // speed demand after throttle shaping and ramp limiting.
    float trim = getInterpolatedTrim(currentOutput);

        
    // Calibration requires identical left/right outputs so that
    // measured RPM differences reflect drivetrain imbalance only.
    // If we are calibrating the trim, do not apply a trim value!
    if(FORCE_OUTPUT > 0.0f )     
    {
        currentOutput = FORCE_OUTPUT;
        trim = 0.0f;      
    }

    float left  = currentOutput;
    float right = currentOutput;

    if (trim > 0.0f)            // +ve trim affects right trak, -ve trim affects left
    {
        right *= (1.0f - trim); // Trim value is percentage to reduce throttle value
    }
    else if (trim < 0.0f)
    {
        left *= (1.0f + trim); // trim is negative
    }
    
    bool forceDACWrite = false;     // If we have just dropped out of a mode pacer delay, force a write to the dac
    if(modePacer.Pace())
    {
        forceDACWrite = true;
    }

    if(modePacer.Running())
    {
        mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
        mcp.setChannelValue(MCP4728_CHANNEL_B, 0);   
        if(currentOutput > 0.0f)
        {
            modePacer.PacerReset();     // Force zero output until throttle is released
        }
        currentOutput = 0.0f;
    }
    else
    {
        setTrackSpeed(TRACK_ID::LEFT, left, forceDACWrite);
        setTrackSpeed(TRACK_ID::RIGHT, right, forceDACWrite);
        newSystemMode &=~ SystemMode::MODECHANGE;
    }

   
    if(serialPacer.Pace())
    {
        Serial.print(F("Throttle :")); Serial.println(currentOutput);
        Serial.print(F("Track L: "));
        Serial.print(left);
        Serial.print(F(" Track R: "));
        Serial.println(right); 
        showMode();
    }

    if(lcdPacer.Pace() && (abs(lastValue - currentOutput) >= 0.01))
    {
        lastValue = currentOutput;
        lcd.setCursor(5,1);
        lcd.print(">");
        barGraph.ShowBargraph(currentOutput);
    }

    // Toggle manoeuvring mode for each press of the mode button. Other mode bits are preserved.
    if(modeSwitch.Pressed())
    {
          newSystemMode ^= SystemMode::SLOWMODE;
    }

}

