#include "KeyHandler.h"

void KeyHandler::HandleEvent(const Falcor::KeyboardEvent& keyEvent)
{
	if (keyEvent.type == Falcor::KeyboardEvent::Type::KeyPressed)
	{
		if (keyEvent.key == Falcor::Input::Key::W)
			Fwd = true;
		else if (keyEvent.key == Falcor::Input::Key::S)
			Bwd = true;
		else if (keyEvent.key == Falcor::Input::Key::A)
			Lft = true;
		else if (keyEvent.key == Falcor::Input::Key::D)
			Rht = true;
	}
	else if (keyEvent.type == Falcor::KeyboardEvent::Type::KeyReleased)
	{
		if (keyEvent.key == Falcor::Input::Key::W)
			Fwd = false;
		else if (keyEvent.key == Falcor::Input::Key::S)
			Bwd = false;
		else if (keyEvent.key == Falcor::Input::Key::A)
			Lft = false;
		else if (keyEvent.key == Falcor::Input::Key::D)
			Rht = false;
	}
}