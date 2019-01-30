/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class BaseIntake extends Subsystem {

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
   * Runs the intake at a given percent power
   * 
   * @param percentOutput The desired percent power for the rollers to run at [-1, 1]
   */
  public abstract void runIntake(double power);

}