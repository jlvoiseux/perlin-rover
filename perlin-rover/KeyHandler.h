#pragma once

#include "Falcor.h"

class KeyHandler
{
public:
	void HandleEvent(const Falcor::KeyboardEvent& keyEvent);

	bool Fwd = false;
	bool Bwd = false;
	bool Lft = false;
	bool Rht = false;

private:
	float mSensitivity = 0.1f;
	float mSpeed = 1.0f;
};

