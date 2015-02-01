#include "StateMachine.h"

StateMachine::StateMachine(ElevatorSystem *s, PincherSystem *p) {
	system = s;
	pinchers = p;
	isCurrentStateComplete = true;
	stateIndex = sizeof(ELEVATOR_STATE_TRANSITIONS) / sizeof(ElevatorState) - 1;
	currentState = ELEVATOR_STATE_TRANSITIONS[stateIndex];

	stateLog = new TextLogger("CurrentStates.log");
}

void StateMachine::PeriodicUpdate(bool buttonInput) {
	UpdateStateMachine(buttonInput);
	DoRunActionForState(currentState, buttonInput);
}

void StateMachine::UpdateStateMachine(bool b) {
	if (isCurrentStateComplete) {
		stateIndex++;
		stateIndex = stateIndex
				% (sizeof(ELEVATOR_STATE_TRANSITIONS) / sizeof(ElevatorState));

		currentState = ELEVATOR_STATE_TRANSITIONS[stateIndex];
		DoPrepActionForState(currentState, b);
		isCurrentStateComplete = false;
	}
}

void StateMachine::DoPrepActionForState(ElevatorState s, bool b) {
	if (s == One_PickupRC) {
		Prep_One_PickupRC(b);
	} else if (s == Two_BackUpFromStack) {
		Prep_Two_BackUpFromStack(b);
	} else if (s == Three_PrepStackPickup) {
		Prep_Three_PrepStackPickup(b);
	} else if (s == Four_HoldStack) {
		Prep_Four_HoldStack(b);
	} else if (s == Five_PrepHPOne) {
		Prep_Five_PrepHPOne(b);
	} else if (s == Six_HPWaitRC) {
		Prep_Six_HPWaitRC(b);
	} else if (s == Seven_HPWaitTote) {
		Prep_Seven_HPWaitTote(b);
	}
}

void StateMachine::DoRunActionForState(ElevatorState s, bool b) {
	bool result;

	if (s == One_PickupRC) {
		result = Run_One_PickupRC(b);
	} else if (s == Two_BackUpFromStack) {
		result = Run_Two_BackUpFromStack(b);
	} else if (s == Three_PrepStackPickup) {
		result = Run_Three_PrepStackPickup(b);
	} else if (s == Four_HoldStack) {
		result = Run_Four_HoldStack(b);
	} else if (s == Five_PrepHPOne) {
		result = Run_Five_PrepHPOne(b);
	} else if (s == Six_HPWaitRC) {
		result = Run_Six_HPWaitRC(b);
	} else if (s == Seven_HPWaitTote) {
		result = Run_Seven_HPWaitTote(b);
	}

	isCurrentStateComplete = result;
}


// First state (Pickup RC) functions

void StateMachine::Prep_One_PickupRC(bool b) {
	stateLog->TextLog("State: Prepped One", "INFO", CODE_STAMP);
	system->SetDiskBreak(false);
	system->StartPIDPosition(0);
}

bool StateMachine::Run_One_PickupRC(bool b) {
	stateLog->TextLog("State: Running One", "INFO", CODE_STAMP);
	system->MoveTo_One_PickupRC();
	// Here, we're externally sucking in the RC

	return (pinchers->BottomProximityTriggered() && system->done);

}

// Second state (Back up from stack) functions

void StateMachine::Prep_Two_BackUpFromStack(bool b) {
	stateLog->TextLog("State: Prepped Two", "INFO", CODE_STAMP);
	system->SetDiskBreak(false);
	system->StartPIDPosition(1);
}

bool StateMachine::Run_Two_BackUpFromStack(bool b) {
	stateLog->TextLog("State: Running Two", "INFO", CODE_STAMP);
	system->MoveTo_Two_BackupFromStack();

	return b;
}

// Third state (Prep Stack Pickup) functions

void StateMachine::Prep_Three_PrepStackPickup(bool b) {
	stateLog->TextLog("State: Prepped Three", "INFO", CODE_STAMP);
	system->SetDiskBreak(false);
	system->StartPIDPosition(2);
}

bool StateMachine::Run_Three_PrepStackPickup(bool b) {

	stateLog->TextLog("State: Running Three", "INFO", CODE_STAMP);
	system->MoveTo_Three_PrepStackPickup();

	return system->done;
}

// Fourth state (Hold stack) functions

void StateMachine::Prep_Four_HoldStack(bool b) {
	stateLog->TextLog("State: Prepped Four", "INFO", CODE_STAMP);
	system->SetDiskBreak(false);
	system->StartPIDPosition(3);
}

bool StateMachine::Run_Four_HoldStack(bool b) {
	stateLog->TextLog("State: Running Four", "INFO", CODE_STAMP);
	system->MoveTo_Four_HoldStack();

	if (system->done) {
		system->SetDiskBreak(true);
	}

	return b;
}

// Fifth state (Prep HP One) functions

void StateMachine::Prep_Five_PrepHPOne(bool b) {
	stateLog->TextLog("State: Prepped Five", "INFO", CODE_STAMP);
	system->SetDiskBreak(false);
	system->StartPIDPosition(4);
}

bool StateMachine::Run_Five_PrepHPOne(bool b) {
	stateLog->TextLog("State: Running Five", "INFO", CODE_STAMP);
	system->MoveTo_Five_PrepHPOne();

	return system->done;
}

// Sixth state (HP Wait RC) functions

void StateMachine::Prep_Six_HPWaitRC(bool b) {
	stateLog->TextLog("State: Prepped Six", "INFO", CODE_STAMP);
	system->SetDiskBreak(false);
	system->StartPIDPosition(5);
}

bool StateMachine::Run_Six_HPWaitRC(bool b) {
	stateLog->TextLog("State: Running Six", "INFO", CODE_STAMP);
	system->MoveTo_Six_HPWaitRC();

	if (system->done) {
		system->SetDiskBreak(true);
	}

	return (pinchers->TopProximityTriggered() && system->done);
}

// Seventh state (HP Wait Tote) functions

void StateMachine::Prep_Seven_HPWaitTote(bool b) {
	stateLog->TextLog("State: Prepped Seven", "INFO", CODE_STAMP);
	system->SetDiskBreak(false);
	system->StartPIDPosition(6);
}

bool StateMachine::Run_Seven_HPWaitTote(bool b) {
	stateLog->TextLog("State: Running Seven", "INFO", CODE_STAMP);
	system->MoveTo_Seven_HPWaitTote();

	if (system->done) {
		system->SetDiskBreak(true);
	}

	return (pinchers->TopProximityTriggered() && system->done);
}
