/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.team670.robot.constants.RobotMap;

/**
 * Represents a tank drive base.
 * 
 * @author shaylandias
 */
public class DriveBase extends Subsystem {


  private CANSparkMax left1, left2, right1, right2;
  private SpeedControllerGroup left, right;
  private DifferentialDrive driveTrain;
  private List<CANSparkMax> leftControllers, rightControllers;
  private List<CANSparkMax> allMotors;

  public DriveBase() {
    // left1 = new CANSparkMax(RobotMap.sparkLeftMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
    // left2 = new CANSparkMax(RobotMap.sparkLeftMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);
    // right1 = new CANSparkMax(RobotMap.sparkRightMotor1, CANSparkMaxLowLevel.MotorType.kBrushless);
    // right2 = new CANSparkMax(RobotMap.sparkRightMotor2, CANSparkMaxLowLevel.MotorType.kBrushless);

    // leftControllers = Arrays.asList(left1,left2);
    // rightControllers = Arrays.asList(right1, right2);
    // allMotors.addAll(leftControllers);
    // allMotors.addAll(rightControllers);
    
    // setMotorsInvert(leftControllers, false);
    // setMotorsInvert(rightControllers, true);

    // left2.follow(left1);
    // right2.follow(right1);

    // left = new SpeedControllerGroup(left1, left2);
    // right = new SpeedControllerGroup(right1, right2);

    // setMotorsBrushless(allMotors);

    // driveTrain = new DifferentialDrive(left, right);

  }

  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton)
   * 
   * @param leftSpeed Speed for left side of drive base [-1, 1]
   * @param rightSpeed Speed for right side of drive base [-1, 1]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  public void initAutonDrive(){
    setMotorsBrakeMode(allMotors, IdleMode.kBrake);
  }

  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton)
   * 
   * @param leftSpeed Speed for left side of drive base [-1, 1]
   * @param rightSpeed Speed for right side of drive base [-1, 1]
   * @param squaredInputs If true, decreases sensitivity at lower inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) { 
    driveTrain.tankDrive(leftSpeed, rightSpeed, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using a curvature drive configuration (wheel)
   * 
   * @param xSpeed The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being right
   * @param isQuickTurn If true enables turning in place and running one side backwards to turn faster
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

   /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with twist)
   * 
   * @param xSpeed The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being right
   * @param isQuickTurn If true, decreases sensitivity at lower inputs
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
    driveTrain.arcadeDrive(xSpeed, zRotation, squaredInputs);
  }

  public int getLeftEncoderPosition(){
    return (int)left1.getEncoder().getPosition();
  }

  /**
   * Gets the position of the front right motor, this encoder gets more positive as it goes forward
   */
  public int getRightEncoderPosition(){
    return (int)right1.getEncoder().getPosition();
  }

  public int getLeftVelocity(){
    return (int)left1.getEncoder().getVelocity();
  }

  public int getRightVelocity(){
    return (int)right1.getEncoder().getVelocity();
  }
  
  /**
   * Inverts a list of motors.
   */
  private void setMotorsInvert(List<CANSparkMax> motorGroup, boolean invert){
    for (CANSparkMax m: motorGroup){
      m.setInverted(invert);
    }
  }

  /**
   * Sets array of motors to be brushless
   */

  private void setMotorsBrushless(List<CANSparkMax> motorGroup){
    for(CANSparkMax m:motorGroup){
      m.setMotorType(CANSparkMaxLowLevel.MotorType.kBrushless);
    }
  }
  /**
   * Sets array of motors to be of a specified mode
   */
  public void setMotorsNeutralMode(IdleMode mode){
    for(CANSparkMax m:allMotors){
      m.setIdleMode(mode);
    }
  }

  /**
   * Sets array of motor to coast mode
   */
  public void setMotorsCoastMode(List<CANSparkMax> motorGroup, IdleMode mode){
    for(CANSparkMax m:motorGroup){
      m.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Sets array of motor to brake mode
   */
  public void setMotorsBrakeMode(List<CANSparkMax> motorGroup, IdleMode mode){
    for(CANSparkMax m:motorGroup){
      m.setIdleMode(IdleMode.kBrake);
    }
  }

  /*
   * Gets the voltage fed into the motor controllers on the left side of the robot
   */
  public double getLeftInputVoltage(){
    double output = left1.getBusVoltage() + left2.getBusVoltage();
    return output;
  }

  /*
   *Get the voltage fed into the motor controllers on the right side of the robot
   */
  public double getRightInputVoltage(){
    double output = right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }
  
  /*
   * Gets the output voltage of the motor controllers on the left side of the robot
   */
  public double getLeftOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output voltage of the motor controllers on the right side of the robot
   */ 
  public double getRightOutputVoltage(){
    double output = right1.getAppliedOutput() + right2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the left side of the robot
   */
  public double getLeftOutputCurrent() {
    double output = left1.getOutputCurrent() + left2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the right side of the robot
   */
  public double getRightOutputCurrent(){
    double output = right1.getOutputCurrent() + right2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the input voltage of all the motor controllers on the robot
   */
  public double getRobotInputVoltage(){
    double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of all the motor controllers on the robot
   */
  public double getRobotOutputVoltage(){
    double output = left1.getAppliedOutput() + left2.getAppliedOutput() + right1.getAppliedOutput() + right2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output current of all the motor controllers on the robot
   */
  public double getRobotOutputCurrent(){
    double output = left1.getOutputCurrent() + left2.getOutputCurrent() + right1.getOutputCurrent() + right2.getOutputCurrent();
    return output;
  }

  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with twist)
   * 
   * @param xSpeed The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being right
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    arcadeDrive(xSpeed, zRotation, false);
  }

  public void stop() {
    tankDrive(0, 0);
  }

/**
 * Return the left CANEncoder Object
 */
  public CANEncoder getLeftEncoder() {
    return left1.getEncoder();

  }

/**
 * Return the right CanEncoder Object
 */
  public CANEncoder getRightEncoder(){
    return right1.getEncoder();
  }

  /**
   * Resets the encoder values to zero.
   */
  public void resetEncoders() {

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
