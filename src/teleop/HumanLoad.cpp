#include "HumanLoad.h"



/*
 *
 	Pinchers running, first tote falls down and sucked in settled on the backstop. Pinchers stay in.
	Stop vertical roller when tote is settled
	Keep running tote accelerator motor up top (tilted wheel)
	Always run
	Come down, lock in and pick up (x3)
	Might be able to come down before tote comes down
	Go all the way down to the bottom and grab both totes
	Lift above carpet
	Retract the locking fingers after the elevator has been lifted up enough to lift the whole stack
	Lowers onto the scoring platform as the whole stack
	-----------------------
	Order of Elevator
	Pick up RC
	Take to stationary handoff when driving to HP
	Come down to HP wait
	Hold for two tote load
	Then go from HP wait to HP load
	Go back up to Stationary Handoff  (+1)
	Then back down to HP wait
	HP load to stationary handoff (+1)
	Then back down to HP wait
	HP load to stationary handoff (+1)
	Last tote comes in, come down to prep pick up two
	Tote comes in, go from prep pick up two to pick up two (+2)
	Drive to scoring platform
	Go to HP load/pick up 5
	Disengage stationary fingers
	Drop to ground
	------------------------
	Positions for the Elevator
	Ground
		-For RC pick up and backing off stack
	Pick up two to go to scoring platform
	HP load position / pick up 5
	HP wait
	Stationary hand-off
	Pick up the whole stack (just above pick up two)
 *
 */

bool preppingToLoad = false;
bool donePrepping = false;
bool liftingRC = false;
bool finishedLifting = false;
bool loading = false;


// Must be called first
void PrepHumanLoad() {
	preppingToLoad = true;
	donePrepping = false;
	liftingRC = false;
	finishedLifting = false;
	loading = false;

}

void LoadRC(ElevatorSystem *elevator, PincherSystem *pinchers) {
	// PICK UP THE RECYCLING CONTAINER
	if (!pinchers->BottomProximityTriggered() && !liftingRC) {
		pinchers->RunRollers();
	}
	else if (pinchers->BottomProximityTriggered() ) {
		liftingRC = true;
	}
	else {
		pinchers->StopPinchers();
	}

	if(liftingRC){
		// GO TO STATIONARY HANDOFF POSITION
		//elevator->

		// IF THERE, SET "liftingRC" TO "false" AND "finishedLifting" TO "true"
		// THEN COME BACK DOWN TO HP WAIT

		//
	}

}

void LoadFromHumanPlayer() {

//	if (!pinchers->BottomProximityTriggered()) {
//			pinchers->RunPinchers();
//		}
//		else if (pinchers->BottomProximityTriggered() ) {
//			pinchers->RunToteAccel();
//		}
//		else {
//			pinchers->StopPinchers();
//		}
}
