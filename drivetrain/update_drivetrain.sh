#!/bin/bash
#
# Updates the drivetrain controllers.

cd $(dirname $0)

export PYTHONPATH=./python

./python/drivetrain.py drivetrain_dog_motor_plant.h \
    drivetrain_dog_motor_plant.cc \
    drivetrain_clutch_motor_plant.h \
    drivetrain_clutch_motor_plant.cc
