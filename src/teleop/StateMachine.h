#include "ElevatorSystem.h"
#include "PincherSystem.h"
#include "logs/TextLogger.h"

#ifndef STATEMACHINE_H
#define STATEMACHINE_H

enum ElevatorState {
	One_PickupRC,
	Two_BackUpFromStack,
	Three_PrepStackPickup,
	Four_HoldStack,
	Five_PrepHPOne,
	Six_HPWaitRC,
	Seven_HPWaitTote
};

// not started, 1, 2, w, 3, 4, 2, w, 3, 4, 2, w, 3, 4, 5, 6, 7, 8, done
// TODO: Change this
const ElevatorState ELEVATOR_STATE_TRANSITIONS[] = {
		Two_BackUpFromStack,
		One_PickupRC,
		Six_HPWaitRC,
		Five_PrepHPOne,
		Seven_HPWaitTote,
		Five_PrepHPOne,
		Seven_HPWaitTote,
		Five_PrepHPOne,
		Seven_HPWaitTote,
		Three_PrepStackPickup,
		Four_HoldStack };

class StateMachine {
	ElevatorSystem *system;
	PincherSystem *pinchers;
	ElevatorState currentState;
	int stateIndex;
	bool isCurrentStateComplete;

	TextLogger *stateLog;

	void UpdateStateMachine(bool b);

	void DoRunActionForState(ElevatorState s, bool b);
	void DoPrepActionForState(ElevatorState s, bool b);

	// All the prep methods
	void Prep_One_PickupRC(bool b);
	void Prep_Two_BackUpFromStack(bool b);
	void Prep_Three_PrepStackPickup(bool b);
	void Prep_Four_HoldStack(bool b);
	void Prep_Five_PrepHPOne(bool b);
	void Prep_Six_HPWaitRC(bool b);
	void Prep_Seven_HPWaitTote(bool b);

	// All the run methods
	bool Run_One_PickupRC(bool b);
	bool Run_Two_BackUpFromStack(bool b);
	bool Run_Three_PrepStackPickup(bool b);
	bool Run_Four_HoldStack(bool b);
	bool Run_Five_PrepHPOne(bool b);
	bool Run_Six_HPWaitRC(bool b);
	bool Run_Seven_HPWaitTote(bool b);

public:
	StateMachine(ElevatorSystem *s, PincherSystem *p);
	void PeriodicUpdate(bool buttonInput);  // this is what would be called from Robot.cpp
};

#endif
