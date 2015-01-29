#include "ElevatorSystem.h"

#ifndef STATEMACHINE_H
#define STATEMACHINE_H


enum ElevatorState {
	NotStarted,
	One,
	Two,
	Three,
	Four,
	Five,
	Six,
	Seven,
	Eight,
	Done,
	Pause
};

// notstarted, 1, 2, w, 3, 4, 2, w, 3, 4, 2, w, 3, 4, 5, 6, 7, 8, done
// TODO: Change this
const ElevatorState ELEVATOR_STATE_TRANSITIONS[] = {NotStarted, One, Pause, Three, Four, Pause, Three, Four, Pause, Seven, Eight, Done};

class StateMachine {
	ElevatorSystem *system;
	ElevatorState currentState;
	int stateIndex;
	bool isCurrentStateComplete;

	void UpdateStateMachine(bool b);
	void DoRunActionForState(ElevatorState s, bool b);
	void DoPrepActionForState(ElevatorState s, bool b);


	// All the prep methods
	void Prep_NotStarted(bool b);
	void Prep_One(bool b);
	void Prep_Two(bool b);
	void Prep_Three(bool b);
	void Prep_Four(bool b);
	void Prep_Five(bool b);
	void Prep_Six(bool b);
	void Prep_Seven(bool b);
	void Prep_Eight(bool b);
	void Prep_Done(bool b);
	void Prep_Pause(bool b);



	// All the run methods
	bool Run_NotStarted(bool b);
	bool Run_One(bool b);
	bool Run_Two(bool b);
	bool Run_Three(bool b);
	bool Run_Four(bool b);
	bool Run_Five(bool b);
	bool Run_Six(bool b);
	bool Run_Seven(bool b);
	bool Run_Eight(bool b);
	bool Run_Done(bool b);
	bool Run_Pause(bool b);






public:
	StateMachine(ElevatorSystem *s);
	void PeriodicUpdate(bool buttonInput);
};








#endif
