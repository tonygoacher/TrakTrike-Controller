#pragma once

class Switch
{
public:
	Switch();
	Switch(uint8_t port);
	void Init(uint8_t port);
	bool Pressed();
	bool IsDown();
private:
	unsigned long m_Timer;
	uint8_t m_Port;
	static const unsigned long sm_DebounceTime = 40;
	enum SwitchState
	{
		OFF,
		PRESSED_HOLD,
		OFF_HOLD,
	};

	SwitchState	m_SwitchState;

	unsigned long Elapsed();
	bool ReadPort();
	void SetTimer();

};