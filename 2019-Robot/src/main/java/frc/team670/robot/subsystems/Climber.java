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
import frc.team670.robot.constants.RobotConstants;
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

  private PIDController frontController;
  private PIDController backController;

  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;

  private boolean backPistonsRetracted;

  public Climber() {    
    backPistons = new WPI_TalonSRX(RobotMap.BACK_CLIMBER_PISTON_CONTROLLER);
    frontPistons = new WPI_TalonSRX(RobotMap.FRONT_CLIMBER_PISTON_CONTROLLER);

    // TODO figure out if these motors need to be inverted.

    frontController = new PIDController(P, I, D, F, new SensorCollection_PIDSource(frontPistons.getSensorCollection()), frontPistons); 
    backController = new PIDController(P, I, D, F, new SensorCollection_PIDSource(frontPistons.getSensorCollection()), backPistons); 
    // TODO implement pitch from NavX instead of yaw - DONE
  }

  public void enablePistonControllers() {
    frontController.setOutputRange(RobotConstants.MINIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);
    frontController.setSetpoint(RobotConstants.FRONT_PISTON_ENCODER_END);
    frontController.setAbsoluteTolerance(RobotConstants.CLIMBER_ENCODER_TOLERANCE);
    frontController.setContinuous(false);
    frontController.enable();

    backController.setOutputRange(RobotConstants.MINIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);
    backController.setSetpoint(RobotConstants.BACK_PISTON_ENCODER_END);
    backController.setAbsoluteTolerance(RobotConstants.CLIMBER_ENCODER_TOLERANCE);
    backController.setContinuous(false);
    backController.enable();
  }

  public void drivePistons(double frontPower, double backPower) {
    frontPistons.set(ControlMode.PercentOutput, frontPower);
    backPistons.set(ControlMode.PercentOutput, backPower);
  }

  public void drivePistonsPID() {
    frontController.setOutputRange(RobotConstants.MINIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);
    backController.setOutputRange(RobotConstants.MINIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);

    // frontController.setOutputRange(frontPistonOutputs[0], frontPistonOutputs[1]);
    // backController.setOutputRange(backPistonOutputs[0], backPistonOutputs[1]);

    // double frontPower = frontController.get();
    // double backPower = backController.get();
    // frontPower += minimumPistonPower;
    // backPower += minimumPistonPower;
    // if (frontPower <= minimumPistonPower) {
    //   frontPower = minimumPistonPower;
    // }
    // if (backPower <= minimumPistonPower) {
    //   backPower = minimumPistonPower;
    // }
    // frontPistons.set(ControlMode.PercentOutput, frontPower);
    // backPistons.set(ControlMode.PercentOutput, backPower);
  }

  public void setFrontPistonOutputRange(double minRange, double maxRange){
    frontController.setOutputRange(minRange, maxRange);
  }

  public void setBackPistonOutputRange(double minRange, double maxRange){
    backController.setOutputRange(minRange, maxRange);
  }

  public void retractFrontPistons() {
    frontController.setSetpoint(RobotConstants.FRONT_PISTON_ENCODER_START);
  }

  public void retractBackPistons() {
    backController.setSetpoint(RobotConstants.BACK_PISTON_ENCODER_START);
    backPistonsRetracted = true;
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

  /**
   * Returns the front PIDController
   * @return the PID controller that controls the front pistons
   */
  public PIDController getFrontController() {
    return frontController;
  }

  /**
   * Returns the back PIDController
   * @return the PID controller that controls the back pistons
   */
  public PIDController getBackController() {
    return backController;
  }

}
// TODO
// the scale is from the outer PID defined by the NAVX
// the innerPIDs only exist because we dont want to overshoot the encoder values
// scale factor is for the output range of the inner PIDs (pistons)
// retract - setpoint is 0, full power backward
// utility command to retract both at a low rate FIRST CHECK: command only if BOTH OF THEM are up (NOT just one)
// this command is just like the climb but down.