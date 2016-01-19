#!/bin/bash
#
# Updates the polydrivetrain controllers and CIM models.

cd $(dirname $0)

export PYTHONPATH=./python

./python/polydrivetrain.py polydrivetrain_dog_motor_plant.h \
    polydrivetrain_dog_motor_plant.cc \
    polydrivetrain_clutch_motor_plant.h \
    polydrivetrain_clutch_motor_plant.cc \
    polydrivetrain_cim_plant.h \
    polydrivetrain_cim_plant.cc
