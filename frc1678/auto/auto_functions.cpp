#include "auto_functions.h"
#include "muan/unitscpp/unitscpp.h"
#include "arm/arm_subsystem.h"

#include <stdio.h>

void AutoFunction::SetUpAutoFunction() {
  // Timer wait_timer = new Timer();
}

void AutoFunction::DeleteAutoFunction() {
  // delete wait_timer;
}

// Test for LemonScript
bool newDriveState = true;
bool AutoFunction::DriveStraight(CitrusRobot* robot, float dist, bool highgear) {
  if (newDriveState) {
    robot->subsystems_.drive.DriveDistance(dist * ft, highgear);
    newDriveState = false;
  }

  if (robot->subsystems_.drive.IsProfileComplete()) {
    newDriveState = true;
    return true;
  } else {
    return false;
  }
}

//TODO(Wesley) Make it so I don't need eyebleach after looking at this function
bool newDriveYoloState = true;
Length yolo_start_dist;
bool AutoFunction::DriveYolo(CitrusRobot* robot, float dist, bool highgear) {
  if (newDriveYoloState) {
    robot->subsystems_.drive.DriveDistance((dist * 1000) * ft, highgear);
    yolo_start_dist = robot->subsystems_.drive.GetDistanceDriven();
    newDriveYoloState = false;
  }

  if (dist >= 0 &&
      robot->subsystems_.drive.GetDistanceDriven() - yolo_start_dist >= dist * ft) {
    newDriveYoloState = true;
    robot->subsystems_.drive.CancelMotionProfile();
    return true;
  } else if (dist < 0 &&
             robot->subsystems_.drive.GetDistanceDriven() - yolo_start_dist <=
                 dist * ft) {
    newDriveYoloState = true;
    robot->subsystems_.drive.CancelMotionProfile();
    return true;
  } else {
    return false;
  }
}

//TODO(Wesley) Make it so I don't need eyebleach after looking at this function
// I feel bad about this code mostly because kyle will mock me if he sees it :P
bool newDriveYoloAtAngleState = true;
Length yolo_angle_start_dist;
bool AutoFunction::DriveYoloAtAngle(CitrusRobot* robot, float dist, float angle, bool highgear) {
  if (newDriveYoloAtAngleState) {
    robot->subsystems_.drive.DriveDistanceAtAngle((dist * 1000) * ft, angle * deg, highgear);
    yolo_start_dist = robot->subsystems_.drive.GetDistanceDriven();
    newDriveYoloAtAngleState = false;
  }

  if (dist >= 0 &&
      robot->subsystems_.drive.GetDistanceDriven() - yolo_start_dist >= dist * ft) {
    newDriveYoloAtAngleState = true;
    robot->subsystems_.drive.CancelMotionProfile();
    return true;
  } else if (dist < 0 &&
             robot->subsystems_.drive.GetDistanceDriven() - yolo_start_dist <=
                 dist * ft) {
    newDriveYoloAtAngleState = true;
    robot->subsystems_.drive.CancelMotionProfile();
    return true;
  } else {
    return false;
  }
}

bool AutoFunction::SetWedge(CitrusRobot* robot, bool up) {
  robot->is_wedge_deployed_ = !up;
  return true;
}

bool newDriveAtAngleState = true;
bool AutoFunction::DriveStraightAtAngle(CitrusRobot* robot, float dist,
                                        float angle, bool highgear) {
  if (newDriveAtAngleState) {
    robot->subsystems_.drive.DriveDistanceAtAngle(dist * ft, angle * deg, highgear);
    newDriveAtAngleState = false;
  }

  if (robot->subsystems_.drive.IsProfileComplete()) {
    newDriveAtAngleState = true;
    return true;
  } else {
    return false;
  }
}

bool newTurnState = true;
bool AutoFunction::PointTurn(CitrusRobot* robot, float angle) {
  if (newTurnState) {
    robot->subsystems_.drive.PointTurn(angle * deg);
    newTurnState = false;
  }

  if (robot->subsystems_.drive.IsProfileComplete()) {
    newTurnState = true;
    return true;
  } else {
    return false;
  }
}

bool newAbsTurnState = true;
bool AutoFunction::AbsolutePointTurn(CitrusRobot* robot, float angle) {
  if (newAbsTurnState) {
    robot->subsystems_.drive.AbsolutePointTurn(angle * deg);
    newAbsTurnState = false;
  }

  if (robot->subsystems_.drive.IsProfileComplete()) {
    newAbsTurnState = true;
    return true;
  } else {
    return false;
  }
}

bool AutoFunction::Shift(CitrusRobot* robot, bool highgear) {
  robot->subsystems_.drive.Shift(highgear);
  return true;
}

Time startTime = -1 * s;
bool AutoFunction::Wait(CitrusRobot* robot, float time) {
  if (startTime < 0 * s) {
    startTime = muan::now();
  }

  if (muan::now() - startTime > time * s) {
    startTime = -1 * s;
    return true;
  }

  return false;
}

Length start_dist;
bool newEncoderWaitState = true;
bool AutoFunction::EncoderWait(CitrusRobot* robot, float dist) {
  if (newEncoderWaitState) {
    newEncoderWaitState = false;
    start_dist = robot->subsystems_.drive.GetDistanceDriven();
    return false;
  }

  if (dist >= 0 &&
      robot->subsystems_.drive.GetDistanceDriven() - start_dist >= dist * ft) {
    newEncoderWaitState = true;
    return true;
  } else if (dist < 0 &&
             robot->subsystems_.drive.GetDistanceDriven() - start_dist <=
                 dist * ft) {
    newEncoderWaitState = true;
    return true;
  } else
    return false;
}

bool AutoFunction::Shoot(CitrusRobot* robot) {
  // Need to set arm position to LONG before calling this shoot
  // Wait for shooter to reach shooting speed
  if (robot->subsystems_.arm.IsDone()) {
    robot->subsystems_.arm.Shoot(false);
    return true;  // shooter->finished();
  }

  return false;
}

bool AutoFunction::RunIntake(CitrusRobot* robot, IntakeStatus run) {
  if (run == OFF) {
    robot->subsystems_.arm.SetIntake(IntakeGoal::OFF);
  } else if (run == UNTIL) {
    robot->subsystems_.arm.SetIntake(IntakeGoal::FORWARD_UNTIL);
  } else if (run == FOREVER) {
    robot->subsystems_.arm.SetIntake(IntakeGoal::FORWARD_FOREVER);
  }
  return true;
}

bool AutoFunction::WaitForBall(CitrusRobot* robot) {
  return robot->subsystems_.arm.BallIntaked();
}

bool newArmPosition = true;
bool AutoFunction::SetArmPosition(CitrusRobot* robot, Position arm_position) {
  if (newArmPosition) {
    switch (arm_position) {
      case LONG:
        robot->subsystems_.arm.GoToLong();
        break;
      case TUCK:
        robot->subsystems_.arm.GoToTuck();
        break;
      case INTAKE:
        robot->subsystems_.arm.GoToIntake();
        break;
      case AUTO_SHOT:
        robot->subsystems_.arm.GoToAutoShot();
        break;
      case DEFENSE:
        robot->subsystems_.arm.GoToDefensive();
        break;
      case TUCK_SPIN:
        robot->subsystems_.arm.GoToTuckSpin();
        break;
      case INTAKE_SPIN:
        robot->subsystems_.arm.GoToIntakeSpin();
        break;
    }

    newArmPosition = false;
  }

  if (robot->subsystems_.arm.IsDone()) {
    newArmPosition = true;
    return true;
  } else {
    return false;
  }
}

bool AutoFunction::CheckArmCalibration(CitrusRobot* robot) {
  return robot->subsystems_.arm.IsCalibrated();
}

bool AutoFunction::DropBall(CitrusRobot* robot) {
  robot->subsystems_.arm.DropBall();
  return true;
}

bool toStartVision = true;
bool AutoFunction::Align(CitrusRobot* robot) {
  if (robot->vision_.RunVision()) {
    return true;
  }

  return false;
}

bool AutoFunction::StopDriving(CitrusRobot* robot) {
  robot->subsystems_.drive.CancelMotionProfile();
  newDriveYoloState = true;
  newDriveState = true;
  newDriveYoloAtAngleState = true;
  newDriveYoloAtAngleState = true;
  newDriveAtAngleState = true;
  newTurnState = true;
  newAbsTurnState = true;
  newArmPosition = true;
  newEncoderWaitState = true;
  return true;
}
