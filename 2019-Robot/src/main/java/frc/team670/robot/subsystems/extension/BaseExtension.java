/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.extension;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.subsystems.TunableSubsystem;

/**
 * Add your docs here.
 */
public abstract class BaseExtension extends Subsystem implements ExtensionInterface, TunableSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public abstract double getForwardSoftLimitInInches();

  public abstract double getReverseSoftLimitInInches();

  public abstract double getMotionMagicSetpoint();

  public abstract void enableBrakeMode();
  
  public abstract void enableCoastMode();

  public abstract boolean getForwardLimitSwitchTripped();

  public abstract boolean getReverseLimitSwitchTripped();

  public abstract double getArbitraryFeedForwardAngleMultiplier();

  public abstract void warnifLimitHit();

  public abstract void resetLimitSwitch();

}
