#ifndef __TWISTTHROTTLE_H__
#define __TWISTTHROTTLE_H__


#include <Arduino.h>

class TwistThrottle {
public:
    // =====================
    // CONFIG STRUCT
    // =====================
    struct Config {
        int pin;

        int minADC;     // full reverse
        int maxADC;     // full forward
        int centerADC;  // idle

        int deadband;   // ADC counts around center

        float filterAlpha;   // 0.0 → 1.0 (lower = smoother)
        float maxRate;       // max change per update (units/sec), 0 = disabled

        int validMin;   // safety range
        int validMax;
    };

    // =====================
    // INIT
    // =====================
    void begin(const Config& cfg) {
        config = cfg;
        pinMode(config.pin, INPUT);

        filtered = config.centerADC;
        lastOutput = 0.0f;
        lastTime = millis();
    }

    // =====================
    // MAIN API
    // =====================
    float GetThrottle() {

        int raw = analogRead(config.pin);

        // ---- Safety check ----
        if (raw < config.validMin || raw > config.validMax) {
            fault = true;
            return 0.0f;
        }

        fault = false;

        // ---- Filtering (IIR) ----
        filtered += (raw - filtered) * config.filterAlpha;

        // ---- Deadband around centre ----
        if (abs(filtered - config.centerADC) < config.deadband) {
            return applyRateLimit(0.0f);
        }

        // ---- Map to -1.0 → +1.0 ----
        float output;

        if (filtered > config.centerADC) {
            float span = config.maxADC - config.centerADC;
            output = (filtered - config.centerADC) / span;
        } else {
            float span = config.centerADC - config.minADC;
            output = (filtered - config.centerADC) / span;
        }

        // Clamp
        output = constrain(output, -1.0f, 1.0f);

        return applyRateLimit(output);
    }

    // =====================
    // STATUS
    // =====================
    bool isFault() const {
        return fault;
    }

private:
    Config config;

    float filtered = 0;
    float lastOutput = 0;
    unsigned long lastTime = 0;

    bool fault = false;

    // =====================
    // RATE LIMITER
    // =====================
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


#endif // __TWISTTHROTTLE_H__