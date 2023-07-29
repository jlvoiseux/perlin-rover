#pragma once

class KeyHandler
{
public:
	void update();

	bool Fwd = false;
	bool Bwd = false;
	bool Lft = false;
	bool Rht = false;

private:
	void KeyPressed();
	void KeyReleased();
	float mSensitivity = 0.1f;
	float mSpeed = 1.0f;
};

