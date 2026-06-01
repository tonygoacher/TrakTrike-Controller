#include "Pacer.h"



Pacer::Pacer(bool autoReset, uint64_t paceTime)
{
    m_autoReset = autoReset;
    m_timeup = !autoReset;
    m_paceTime = paceTime;
    m_running = !m_timeup;
}

bool Pacer::Pace()
{
    bool timeUp = m_timeup;
    if(!timeUp && millis() >= m_nextTime)
    {
        timeUp = true;
        m_timeup = true;
        if(m_autoReset)
        {
            PacerReset();    
        }
        else
        {
            m_running = false;
        }

    }   
    return timeUp;
}

void Pacer::SetPace(uint64_t newPace)
{
   m_paceTime = newPace; 
   if(m_autoReset)
   {
        PacerReset();
   }
  
}

void Pacer::PacerReset()
{
    m_timeup = false;
    m_nextTime = millis() + m_paceTime;    
    m_running = true;
}