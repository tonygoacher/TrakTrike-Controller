#include <Arduino.h>
#include "NanoThrottle.h"



NanoThrottle::NanoThrottle(uint8_t port)
{
   m_port = port; 
}

float NanoThrottle::GetThrottle()
{
    return analogRead(m_port)/ MaxADC;
}
