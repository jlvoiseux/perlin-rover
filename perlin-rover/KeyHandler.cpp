#include "KeyHandler.h"

#include "raylib.h"

void KeyHandler::KeyPressed()
{
	if (IsKeyPressed(KEY_W))
		Fwd = true;
	else if (IsKeyPressed(KEY_S))
		Bwd = true;
	else if (IsKeyPressed(KEY_A))
		Lft = true;
	else if (IsKeyPressed(KEY_D))
		Rht = true;
}


void KeyHandler::KeyReleased()
{
	if (IsKeyReleased(KEY_W))
		Fwd = false;
	else if (IsKeyReleased(KEY_S))
		Bwd = false;
	else if (IsKeyReleased(KEY_A))
		Lft = false;
	else if (IsKeyReleased(KEY_D))
		Rht = false;
}


void KeyHandler::update()
{
	KeyPressed();
	KeyReleased();
}