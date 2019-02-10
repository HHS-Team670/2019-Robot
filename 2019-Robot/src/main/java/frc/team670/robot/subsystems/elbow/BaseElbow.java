/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.elbow;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.subsystems.RotatingSubsystem;

/**
 * Add your docs here.
 */
public abstract class BaseElbow extends RotatingSubsystem implements ElbowInterface {

  public BaseElbow(TalonSRX rotatorTalon, double arbitraryFeedForwardConstant, int forwardSoftLimit, int reverseSoftLimit, boolean timeout, int quadEncoerMin, int quadEncoderMax, int continuousCurrentLimit, int peakCurrentLimit, int offsetFromEncoderZero){
    super(rotatorTalon, arbitraryFeedForwardConstant, forwardSoftLimit, reverseSoftLimit, timeout, quadEncoerMin, quadEncoderMax, continuousCurrentLimit, peakCurrentLimit, offsetFromEncoderZero);
  }

  @Override
  protected void initDefaultCommand() {
    
  }

}
