/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.team670.robot.commands.tuning.RotatingSubsystem;

public abstract class BaseIntake extends RotatingSubsystem {

  @Override
  public abstract void initDefaultCommand();

   /**
   * Should set the setpoint for the Motion Magic on the intake
   */
  public abstract void setMotionMagicSetpoint(double intakeTicks);

  /**
   * Should return the setpoint for the motion magic on the base motor
   */
  public abstract double getMotionMagicSetpoint();

  /**
   * Should return the setpoint coordinates for the motion magic on the base motor
   */
  public abstract Point2D.Double getMotionMagicDestinationCoordinates();

  public abstract void setRotatorNeutralMode(NeutralMode mode);

  /**
   * Returns the tick value of the base motor
   */
  public abstract int getIntakePositionInTicks();

  /**
   * Returns the intake angle in degrees
   */
  public abstract double getIntakeAngleInDegrees();

  /** 
   * Returns the x, y coordinates of the top of the intake
   */
  public abstract Point2D.Double getIntakeCoordinates();


  /**
   * Runs the intake at a given percent power
   * 
   * @param percentOutput The desired percent power for the rollers to run at [-1, 1]
   */
  public abstract void runIntake(double power);

}