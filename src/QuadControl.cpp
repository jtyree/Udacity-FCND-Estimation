#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   desCollectiveThrust: desired collective thrust [N]
  //   desMoment: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of desMoment via e.g. desMoment.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

	float p_bar, q_bar, r_bar;
	float l_right_angle;
	float m1, m2, m3, m4;

	l_right_angle = L / sqrt(2.0f);
	p_bar = momentCmd.x / l_right_angle;
	q_bar = momentCmd.y / l_right_angle;
	r_bar = momentCmd.z / kappa;

	// For testing
	// collThrustCmd = 4.905f;
	// p_bar = 0.0f;
	// q_bar = 0.0f;
	// r_bar = 0.0f;

	m1 = (collThrustCmd + p_bar + q_bar - r_bar) / 4.0f;
	m2 = (collThrustCmd - p_bar + q_bar + r_bar) / 4.0f;
	m3 = (collThrustCmd + p_bar - q_bar + r_bar) / 4.0f;
	m4 = (collThrustCmd - p_bar - q_bar - r_bar) / 4.0f;

	cmd.desiredThrustsN[0] = CONSTRAIN(m1, minMotorThrust, maxMotorThrust); // front left
	cmd.desiredThrustsN[1] = CONSTRAIN(m2, minMotorThrust, maxMotorThrust); // front right
	cmd.desiredThrustsN[2] = CONSTRAIN(m3, minMotorThrust, maxMotorThrust); // rear left
	cmd.desiredThrustsN[3] = CONSTRAIN(m4, minMotorThrust, maxMotorThrust); // rear right

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F error;
  V3F moi;

  moi.x = Ixx;
  moi.y = Iyy;
  moi.z = Izz;

  error = pqrCmd - pqr;
  momentCmd = error * kpPQR * moi;
 

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float b_x_dot;
  float b_y_dot;
  float R02_command;
  float R12_command;
  float c;

  // Convert thrust to acceleration
  c = -collThrustCmd / mass;

  // For testing
  // printf("Thrust=%f\n", collThrustCmd);

  // NOTE for future: c = 0 could make it blow up!

  // limit tilt
  R02_command = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
  R12_command = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);

  // commands from kp and error
  b_x_dot = kpBank * (R02_command - R(0, 2));
  b_y_dot = kpBank * (R12_command - R(1, 2));

  // convert to pqr
  pqrCmd.x = (R(1, 0) * b_x_dot - R(0, 0) * b_y_dot) / R(2, 2);
  pqrCmd.y = (R(1, 1) * b_x_dot - R(0, 1) * b_y_dot) / R(2, 2);

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float z_error, z_dot_error;
  float u1_bar;
  float int_limit = 1.0f;

  // printf("velZCmd=%f\n", velZCmd);

  // z velocity positive is descending
  if (velZCmd > maxDescentRate) {
	  velZCmd = maxDescentRate;
  }

  // z velocity negative is ascending
  if (velZCmd < -maxAscentRate) {
	  velZCmd = -maxAscentRate;
  }
  
  z_error = posZCmd - posZ;
  z_dot_error = velZCmd - velZ;

  // calc integral
  integratedAltitudeError += z_error * dt;

  // limit integral so it can't climb forever (beyond the performance of the motors)
  integratedAltitudeError = fmodf(integratedAltitudeError, int_limit);

  // The python model had 9.81 coming in as FF, this one does not so add it in here
  u1_bar = kpPosZ * z_error + kpVelZ * z_dot_error + KiPosZ * integratedAltitudeError + accelZCmd - 9.81f;

  thrust = -(u1_bar / R(2, 2)) * mass;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmd: desired acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you cap the horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmd.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  V3F pos_err, vel_err;
  V3F p_term, d_term;
  V3F pos_dot_dot_cmd;

  // Cap max speed cmd
  velCmd.x = fmodf(velCmd.x, maxSpeedXY);
  velCmd.y = fmodf(velCmd.y, maxSpeedXY);

  // Cap max accel cmd
  accelCmd.x = fmodf(accelCmd.x, maxAccelXY);
  accelCmd.y = fmodf(accelCmd.y, maxAccelXY);

  pos_err = posCmd - pos;
  vel_err = velCmd - vel;

  pos_dot_dot_cmd = pos_err * kpPosXY + vel_err * kpVelXY + accelCmd;
  
  pos_dot_dot_cmd.z = 0;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pos_dot_dot_cmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  float yaw_err;

  yaw_err = yawCmd - yaw;
  if (yaw_err >= 3.14f)
	  yaw_err -= 3.14f;
  else if (yaw_err <= -3.14f)
	  yaw_err += 3.14f;

  yawRateCmd = kpYaw * yaw_err;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
