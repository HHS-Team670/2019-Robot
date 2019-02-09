/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public abstract class BaseIntake extends RotatingSubsystem {
  
  public BaseIntake(TalonSRX rotatorTalon, double arbitraryFeedForwardConstant, int forwardSoftLimit, int reverseSoftLimit, boolean timeout, int quadEncoderMin, int quadEncoderMax, int continousCurrentLimit, int peakCurrentLimit){
    super(rotatorTalon, arbitraryFeedForwardConstant, forwardSoftLimit, reverseSoftLimit, timeout, quadEncoderMin, quadEncoderMax, continousCurrentLimit, peakCurrentLimit);
  }

  @Override
  public abstract void initDefaultCommand();

  /**
   * Should return the setpoint coordinates for the motion magic on the base motor
   */
  public abstract Point2D.Double getMotionMagicDestinationCoordinates();

  public abstract void setRotatorNeutralMode(NeutralMode mode);
  
  /** 
   * Returns the x, y coordinates of the top of the intake
   */
  public abstract Point2D.Double getIntakeCoordinates();


  /**
   * Runs the intake at a given percent power
   * 
   * @param percentOutput The desired percent power for the rollers to run at [-1, 1]
   */
  public abstract void runIntake(double power, boolean runningIn);

}