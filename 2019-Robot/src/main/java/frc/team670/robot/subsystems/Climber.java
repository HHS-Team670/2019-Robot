/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.SensorCollection_PIDSource;

/**
 * Represents the climbing mechanism on the robot.
 * @author shaylandias
 */
public class Climber extends Subsystem {

  // Motor that drives the two pistons in the back of the robot
  private WPI_TalonSRX backPistons;
  // Motor that drives the two pistons in the front of the robot. May be split into two controllers.
  private WPI_TalonSRX frontPistons;

  private Encoder frontEncoder, backEncoder;

  private PIDController frontController;
  private PIDController backController;

  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
  private double frontPistonControllerLowerOutput = -0.75, frontPistonControllerUpperOutput = 0.75; // [-1, 1]0.75
  private double backPistonControllerLowerOutput = -0.75, backPistonControllerUpperOutput = 0.75; // [-1, 1]0.75
  private boolean backPistonsRetracted;

  //TODO set these
  private int frontEncoderEnd = 0; // where the front encoder should be when it is finished climbing
  private int backEncoderEnd = 0; // """" back encoder """"
  private int finishTolerance = 10; // return true for isFinished if the encoder value is within this many ticks of the endpoint
  private final double minimumPistonPower = 0.1;



  public Climber() {
    backPistons = new WPI_TalonSRX(RobotMap.backClimberPistonController);
    frontPistons = new WPI_TalonSRX(RobotMap.frontClimberPistonController);

    // TODO figure out if these motors need to be inverted.

    frontController = new PIDController(P, I, D, F, new SensorCollection_PIDSource(frontPistons.getSensorCollection()), frontPistons); 
    backController = new PIDController(P, I, D, F, new SensorCollection_PIDSource(frontPistons.getSensorCollection()), backPistons); 
    // TODO implement pitch from NavX instead of yaw

    frontController.setOutputRange(frontPistonControllerLowerOutput, frontPistonControllerUpperOutput);
    frontController.setSetpoint(frontEncoderEnd);
    frontController.setContinuous(true);

    backController.setOutputRange(backPistonControllerLowerOutput, backPistonControllerUpperOutput);
    backController.setSetpoint(backEncoderEnd);
    backController.setContinuous(true);

  }

  /**
   * Drives pistons downwards with the given powers.
   */
  public void drivePistons(double frontPower, double backPower) {
    
    frontPistons.set(ControlMode.PercentOutput, frontPower);
    backPistons.set(ControlMode.PercentOutput, backPower);
  }

  public void drivePistonsPID() {
    double frontPower = frontController.get();
    double backPower = backController.get();
    frontPower += minimumPistonPower;
    backPower += minimumPistonPower;
    if (frontPower <= minimumPistonPower) {
      frontPower = minimumPistonPower;
    }
    if (backPower <= minimumPistonPower) {
      backPower = minimumPistonPower;
    }
    frontPistons.set(ControlMode.PercentOutput, frontPower);
    backPistons.set(ControlMode.PercentOutput, backPower);
  }

  public void retractFrontPistons() {
    frontController.setSetpoint(0); // TODO set to the actual value
    driveFrontPistons(0);
  }

  public void retractBackPistons() {
    backController.setSetpoint(0); // TODO set to the actual value
    driveBackPistons(0);
    backPistonsRetracted = true;
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
    // wait for the expected method to be called and then retract back pistons

    return backPistonsRetracted;
    //return frontController.onTarget() && backController.onTarget();

    // return MathUtils.doublesEqual(frontEncoder.get(), frontEncoderEnd, finishTolerance)
    //   && MathUtils.doublesEqual(backEncoder.get(), backEncoderEnd, finishTolerance);
  }

  @Override
  public void initDefaultCommand() {
  }

  public PIDController getFrontController() {
    return frontController;
  }

  public PIDController getBackController() {
    return backController;
  }

  public WPI_TalonSRX getFrontPistons(){
    return frontPistons;
  }

  public WPI_TalonSRX getBackPistons(){
    return backPistons;
  }

  public double getFrontMinOutput(){
    return frontPistonControllerLowerOutput;
  }

  public double getFrontMaxOutput(){
    return frontPistonControllerUpperOutput;
  }

  public double getBackMinOutput(){
    return backPistonControllerLowerOutput;
  }

  public double getBackMaxOutput(){
    return backPistonControllerUpperOutput;
  }

}
// TODO
// the scale is from the outer PID defined by the NAVX
// the innerPIDs only exist because we dont want to overshoot the encoder values
// scale factor is for the output range of the inner PIDs (pistons)
// retract - setpoint is 0, full power backward
// utility command to retract both at a low rate FIRST CHECK: command only if BOTH OF THEM are up (NOT just one)
// this command is just like the climb but down.