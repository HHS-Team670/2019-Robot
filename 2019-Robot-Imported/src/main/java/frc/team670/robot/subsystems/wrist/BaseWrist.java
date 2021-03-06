/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.subsystems.RotatingSubsystem;

/**
 * Add your docs here.
 */
public abstract class BaseWrist extends RotatingSubsystem implements WristInterface {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public BaseWrist(TalonSRX rotatorTalon, double arbitrary_feedforward_constant, int forward_soft_limit, int reverse_soft_limit, boolean timeout, int QUAD_ENCODER_MIN, int QUAD_ENCODER_MAX, int CONTINUOUS_CURRENT_LIMIT, int PEAK_CURRENT_LIMIT, int offsetFromEncoderZero){
    super(rotatorTalon, arbitrary_feedforward_constant, forward_soft_limit, reverse_soft_limit, timeout, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, offsetFromEncoderZero);
  }

  public abstract double getForwardSoftLimitAngle();

  public abstract double getReverseSoftLimitAngle();


  @Override
  public void initDefaultCommand() {

  }

}
