/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the climbing mechanism on the robot.
 * @author shaylandias
 */
public class Climber extends Subsystem {

  // Motor that drives the two pistons in the back of the robot
  private TalonSRX backPistons;
  // Motor that drives the two pistons in the front of the robot. May be split into two controllers.
  private TalonSRX frontPistons;

  public Climber() {
    backPistons = new TalonSRX(RobotMap.BACK_CLIMBER_PISTON_CONTROLLER);
    frontPistons = new TalonSRX(RobotMap.FRONT_CLIMBER_PISTON_CONTROLLER);

    // TODO figure out if these motors need to be inverted.
  }

  /**
   * Drives the pistons downwards with the given power.
   */
  public void drivePistons(double power) {
    backPistons.set(ControlMode.PercentOutput, power);
    frontPistons.set(ControlMode.PercentOutput, power);
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
