#include "StateMachine.h"

StateMachine::StateMachine(ElevatorSystem *s, PincherSystem *p) {
	system = s;
	pinchers = p;
	isCurrentStateComplete = false;
	stateIndex = 0;
	currentState = ELEVATOR_STATE_TRANSITIONS[0];
}

void StateMachine::PeriodicUpdate(bool buttonInput) {
	UpdateStateMachine(buttonInput);
	DoRunActionForState(currentState, buttonInput);
}

void StateMachine::UpdateStateMachine(bool b) {
	if(isCurrentStateComplete) {
		stateIndex++;
		stateIndex = stateIndex % (sizeof(ELEVATOR_STATE_TRANSITIONS) / sizeof(ElevatorState));

		currentState = ELEVATOR_STATE_TRANSITIONS[stateIndex];
		DoPrepActionForState(currentState, b);
		isCurrentStateComplete = false;
	}
}


void StateMachine::DoPrepActionForState(ElevatorState s, bool b) {
	if(s == NotStarted) {
		Prep_NotStarted(b);
	}
	else if(s == One) {
		Prep_One(b);
	}
	else if(s == Two) {
		Prep_Two(b);
	}
	else if(s == Three) {
		Prep_Three(b);
	}
	else if(s == Four) {
		Prep_Four(b);
	}
	else if(s == Five) {
		Prep_Five(b);
	}
	else if(s == Six) {
		Prep_Six(b);
	}
	else if(s == Seven) {
		Prep_Seven(b);
	}
	else if(s == Eight) {
		Prep_Eight(b);
	}
	else if(s == Done) {
		Prep_Done(b);
	}
	else if(s == Pause) {
		Prep_Pause(b);
	}
}

void StateMachine::DoRunActionForState(ElevatorState s, bool b) {
	bool result;

	if(s == NotStarted) {
		result = Run_NotStarted(b);
	}
	else if(s == One) {
		result = Run_Two(b);
	}
	else if(s == Two) {
		result = Run_Three(b);
	}
	else if(s == Four) {
		result = Run_Four(b);
	}
	else if(s == Five) {
		result = Run_Five(b);
	}
	else if(s == Six) {
		result = Run_Six(b);
	}
	else if(s == Seven) {
		result = Run_Seven(b);
	}
	else if(s == Eight) {
		result = Run_Eight(b);
	}
	else {

	}

	isCurrentStateComplete = result;
}


// Not started functions

void StateMachine::Prep_NotStarted(bool b) {

}

bool StateMachine::Run_NotStarted(bool b) {
	return b;
}


// First state functions

void StateMachine::Prep_One(bool b) {
	system->StartPIDPosition(0);
}

bool StateMachine::Run_One(bool b) {
//	if(pinchers->ProximityTriggered()) {
//
//	}
}

// Second state functions

void StateMachine::Prep_Two(bool b) {

}

bool StateMachine::Run_Two(bool b) {

}

// Third state functions

void StateMachine::Prep_Three(bool b) {

}

bool StateMachine::Run_Three(bool b) {

}

// Fourth state functions

void StateMachine::Prep_Four(bool b) {

}

bool StateMachine::Run_Four(bool b) {

}

// Fifth state functions

void StateMachine::Prep_Five(bool b) {

}

bool StateMachine::Run_Five(bool b) {

}

// Sixth state functions

void StateMachine::Prep_Six(bool b) {

}

bool StateMachine::Run_Six(bool b) {

}

// Seventh state functions

void StateMachine::Prep_Seven(bool b) {

}

bool StateMachine::Run_Seven(bool b) {

}

// Eight state functions

void StateMachine::Prep_Eight(bool b) {

}

bool StateMachine::Run_Eight(bool b) {

}

// Wait state functions

void StateMachine::Prep_Pause(bool b) {

}

bool StateMachine::Run_Pause(bool b) {

}

// Pause state functions

void StateMachine::Prep_Done(bool b) {

}

bool StateMachine::Run_Done(bool b) {

}


