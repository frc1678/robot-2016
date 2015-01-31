#include "StateMachine.h"



StateMachine::StateMachine(ElevatorSystem *s, PincherSystem *p) {
	system = s;
	pinchers = p;
	isCurrentStateComplete = false;
	stateIndex = 0;
	currentState = ELEVATOR_STATE_TRANSITIONS[0];

	stateLog = new TextLogger("CurrentStates.log");
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
	else if(s == Done) {
		result = Run_Done(b);
	}
	else if(s == Pause) {
		result = Run_Pause(b);
	}

	isCurrentStateComplete = result;
}


// Not started functions

void StateMachine::Prep_NotStarted(bool b) {

	stateLog->TextLog("State: Prepped Not Started", "INFO", CODE_STAMP);
}

bool StateMachine::Run_NotStarted(bool b) {
	stateLog->TextLog("State: Running Not Started", "INFO", CODE_STAMP);
	return b && pinchers->BottomProximityTriggered();
}


// First state functions

void StateMachine::Prep_One(bool b) {
	stateLog->TextLog("State: Prepped One", "INFO", CODE_STAMP);
	system->StartPIDPosition(0);
}

bool StateMachine::Run_One(bool b) {
	stateLog->TextLog("State: Running One", "INFO", CODE_STAMP);
	system->MoveToStationaryPosition();

	return system->done;

}

// Second state functions, not being used right now

void StateMachine::Prep_Two(bool b) {

}

bool StateMachine::Run_Two(bool b) {

}

// Third state functions

void StateMachine::Prep_Three(bool b) {
	stateLog->TextLog("State: Prepped Three", "INFO", CODE_STAMP);
	system->StartPIDPosition(1);
}

bool StateMachine::Run_Three(bool b) {

	stateLog->TextLog("State: Running Three", "INFO", CODE_STAMP);
	system->MoveToHPLoadOne();

	return system->done;
}

// Fourth state functions

void StateMachine::Prep_Four(bool b) {
	stateLog->TextLog("State: Prepped Four", "INFO", CODE_STAMP);
	system->StartPIDPosition(2);
}

bool StateMachine::Run_Four(bool b) {

	stateLog->TextLog("State: Running Four", "INFO", CODE_STAMP);

	system->MoveToStationaryPosition();

	return system->done;
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
	stateLog->TextLog("State: Prepped Seven", "INFO", CODE_STAMP);
	system->StartPIDPosition(3);
}

bool StateMachine::Run_Seven(bool b) {
	stateLog->TextLog("State: Running Seven", "INFO", CODE_STAMP);

	system->MoveToHPLoadTwo();

	return system->done;
}

// Eight state functions

void StateMachine::Prep_Eight(bool b) {

	stateLog->TextLog("State: Prepped Eight", "INFO", CODE_STAMP);

	system->StartPIDPosition(4);
}

bool StateMachine::Run_Eight(bool b) {
	stateLog->TextLog("State: Running Eight", "INFO", CODE_STAMP);

	system->MoveToScoringPosition();
}

// Wait state functions

void StateMachine::Prep_Pause(bool b) {
	stateLog->TextLog("State: Prepped Pause", "INFO", CODE_STAMP);
}

bool StateMachine::Run_Pause(bool b) {
	stateLog->TextLog("State: Running Pause", "INFO", CODE_STAMP);

	return pinchers->TopProximityTriggered(); // TODO: make sure this does what we want it to
}

// Pause state functions

void StateMachine::Prep_Done(bool b) {
	stateLog->TextLog("State: Prepped Done", "INFO", CODE_STAMP);

	system->StartPIDPosition(4);
}

bool StateMachine::Run_Done(bool b) {
	stateLog->TextLog("State: Running Done", "INFO", CODE_STAMP);

	system->MoveToGround();

	return system->done;
}


