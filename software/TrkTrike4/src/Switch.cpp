#include <Arduino.h>

#define	SWITCH0	2



#include <Arduino.h>

#include "Switch.h"


Switch::Switch()
{

}

Switch::Switch(uint8_t port)
{
	Init(port);
}

unsigned long Switch::Elapsed()
{
	return millis() - m_Timer;
}

void Switch::Init(uint8_t port)
{
	m_Port = port;
	pinMode(port, INPUT_PULLUP);
	m_SwitchState = Switch::OFF;
}

bool Switch::ReadPort()
{
	return digitalRead(m_Port) == LOW;
}

void Switch::SetTimer()
{
	m_Timer = millis();
}

bool Switch::IsDown()
{
	return ReadPort();
}

bool Switch::Pressed()
{
	bool pressed = false;
	switch (m_SwitchState)
	{
		case Switch::OFF:
		{
			if (ReadPort())
			{
				pressed = true;
				m_SwitchState = Switch::PRESSED_HOLD;
				SetTimer();
			}
		}
		break;

		case Switch::PRESSED_HOLD:
		{
			if (!ReadPort() && Elapsed() > sm_DebounceTime)
			{
				m_SwitchState = Switch::OFF_HOLD;
				SetTimer();
			}
		}
		break;

		case Switch::OFF_HOLD:
		{
			if (!ReadPort() && Elapsed() > sm_DebounceTime)	
			{
				m_SwitchState = Switch::OFF;
			}
		}
		break;
	}

	return pressed;
}