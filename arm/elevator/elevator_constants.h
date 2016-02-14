#ifndef ELEVATOR_ELEVATOR_CONSTANTS_H_
#define ELEVATOR_ELEVATOR_CONSTANTS_H_

#include "unitscpp/unitscpp.h"
#include "Eigen/Core"
#include "muan/control/control_utils.h"

namespace ElevatorConstants {

//Constants

// Motor constants
// Torque Constanti
auto stall_current = 134 * A;
auto stall_torque = 0.71 * N * m;
auto free_speed = 18730 * rev / (60*s);
decltype(N*m/A) K_t = stall_torque/stall_current;
// Voltage Constant
decltype(V/(rad/s)) K_v = 12*V / free_speed;
// Motor resistance
auto R = 12 * V / stall_current;
// Gear Ratio
double G = 1/40.0;
// Pulley radius
auto r = (1.43/2)*in;
// Mass of elevator carriage
auto mass = 18*kg;

//Friction thingy for matrices
double kF = ((K_t*K_v)/(G*G*R*r*r*mass)).to(1/s);
//Voltage thingy for matrices
double kM = ((K_t)/(G*R*r*mass)).to(N*m/(V*m*kg)); //matrices (x_hat = A*x + B*u)

auto A_c = muan::as_matrix<2, 2>({{0, 1}, {0, -kF}});
auto B_c = muan::as_matrix<2, 1>({{0}, {kM}});
auto C_c = muan::as_matrix<1, 2>({{1, 0}});

auto L = muan::as_matrix<2, 1>({{2.056}, {-.08}});
auto K = muan::as_matrix<1, 2>({{254.971, 20.1678}});

auto maxDisplacement = 0.005 * m;
}

#endif
