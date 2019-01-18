/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Represents the climbing mechanism on the robot.
 * @author shaylandias
 */
public class Climber extends Subsystem {

  // Motor that drives the two pistons in the back of the robot
  private TalonSRX backPistons;
  // Motor that drives the two pistons in the front of the robot. May be split into two controllers.
  private TalonSRX frontPistons;

  private Encoder frontEncoder, backEncoder;

  //TODO set these
  private int frontEncoderEnd = 0; // where the front encoder should be when it is finished climbing
  private int backEncoderEnd = 0; // """" back encoder """"
  private int finishTolerance = 10; // return true for isFinished if the encoder value is within this many ticks of the endpoint

  public Climber() {
    backPistons = new TalonSRX(RobotMap.backClimberPistonController);
    frontPistons = new TalonSRX(RobotMap.frontClimberPistonController);

    // TODO figure out if these motors need to be inverted.
  }

  /**
   * Drives pistons downwards with the given powers.
   */
  public void drivePistons(double frontPower, double backPower) {
    frontPistons.set(ControlMode.PercentOutput, frontPower);
    backPistons.set(ControlMode.PercentOutput, backPower);
  }

  /**
   * Drives front pistons downwards with given power
   */
  private void driveFrontPistons(double power) {
    frontPistons.set(ControlMode.PercentOutput, power);
  }

  /**
   * Drives back pistons downwards with given power
   */
  private void driveBackPistons(double power) {
    backPistons.set(ControlMode.PercentOutput, power);
  }

  public boolean isFinished() {
    return MathUtils.doublesEqual(frontEncoder.get(), frontEncoderEnd, finishTolerance)
      && MathUtils.doublesEqual(backEncoder.get(), backEncoderEnd, finishTolerance);
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
