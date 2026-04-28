#ifndef __NANOTHROTTLE_H__
#define __NANOTHROTTLE_H__
#include <stdint.h>

#include "IThrottle.h"

// Throttle class to return 0.0-1.0 for Arduino A/D

class NanoThrottle : public IThrottle
{
public:
    NanoThrottle(uint8_t port);
    float GetThrottle() override;
private:
    static const int MaxADC = 1023;
    uint8_t m_port;    
};



#endif // __NANOTHROTTLE_H__