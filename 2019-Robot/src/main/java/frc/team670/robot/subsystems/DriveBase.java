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
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team670.robot.commands.drive.XboxRocketLeagueDrive;
import frc.team670.robot.utils.functions.MathUtils;

import frc.team670.robot.constants.RobotMap;

/**
 * Represents a tank drive base.
 * 
 * @author shaylandias
 */
public class DriveBase extends Subsystem {

  private static final int velocityPIDSlot = 1, encodersPIDSlot = 2;

  private CANSparkMax left1, left2, right1, right2;
  private SpeedControllerGroup left, right;
  private DifferentialDrive driveTrain;
  private List<CANSparkMax> leftControllers, rightControllers;
  private List<CANSparkMax> allMotors;
  private double P, I, D, FF;

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

    left1.getPIDController().setP(P, encodersPIDSlot);
    left1.getPIDController().setI(I, encodersPIDSlot);
    left1.getPIDController().setD(D, encodersPIDSlot);
    left1.getPIDController().setFF(FF, encodersPIDSlot);
    left1.getPIDController().setOutputRange(-1, 1);

    right1.getPIDController().setP(P, encodersPIDSlot);
    right1.getPIDController().setI(I, encodersPIDSlot);
    right1.getPIDController().setD(D, encodersPIDSlot);
    right1.getPIDController().setFF(FF, encodersPIDSlot);
    right1.getPIDController().setOutputRange(-1, 1);

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

  /**
   * Stops the motors on the drive base (sets them to 0).
   */
  public void stop() {
    tankDrive(0, 0);
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

/**
 * Return the left CANEncoder Object
 * @deprecated Do not access these for PID Controllers anymore, use the internal PIDControllers for the SparkMAX motors.
 */
  public CANEncoder getLeftEncoder() {
    return left1.getEncoder();

  }

/**
 * Return the right CanEncoder Object
 * @deprecated Do not access these for PID Controllers anymore, use the internal PIDControllers for the SparkMAX motors.
 */
  public CANEncoder getRightEncoder(){
    return right1.getEncoder();
  }

  /**
   * Sets the PIDControllers setpoints for the left and right side motors to the given positions in ticks forward.
   * @param deltaLeft The desired change in left position in encoder ticks
   * @param deltaRight The desired change in right position in encoder ticks
   */
  public void setEncodersControl(double deltaLeft, double deltaRight) {
    left1.getPIDController().setReference(left1.getEncoder().getPosition() + deltaLeft, ControlType.kPosition, encodersPIDSlot);
    right1.getPIDController().setReference(right1.getEncoder().getPosition() + deltaRight, ControlType.kPosition, encodersPIDSlot);
  }

  /**
   * Sets the velocities of the left and right motors of the robot.
   * @param leftVel Velocity for left motors in inches/sec
   * @param rightVel Velocity for right motors in inches/sec
   */
  public void setVelocityControl(double leftVel, double rightVel) {
    leftVel = MathUtils.convertInchesPerSecondToDriveBaseRoundsPerMinute(MathUtils.convertInchesToDriveBaseTicks(leftVel));
    rightVel = MathUtils.convertInchesPerSecondToDriveBaseRoundsPerMinute(MathUtils.convertInchesToDriveBaseTicks(rightVel));
    left1.getPIDController().setReference(leftVel, ControlType.kVelocity, velocityPIDSlot);
    right1.getPIDController().setReference(rightVel, ControlType.kVelocity, velocityPIDSlot);
  }

    /**
   * Gets the encoder position of the front left motor in ticks.
   */
  public int getLeftEncoderPosition(){
    return (int)left1.getEncoder().getPosition();
  }

  /**
   * Gets the encoder position of the front right motor in ticks, this encoder gets more positive as it goes forward
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
   * Gets the input voltage of all the drivebase motor controllers on the robot
   */
  public double getDriveBaseInputVoltage(){
    double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of all the drivebase motor controllers on the robot
   */
  public double getDriveBaseOutputVoltage(){
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

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new XboxRocketLeagueDrive());
  }

  public CANSparkMax getLeft1(){
    return left1;
  }
  public CANSparkMax getLeft2(){
    return left2;
  }
  public CANSparkMax getRight1(){
    return right1;
  }
  public CANSparkMax getRight2(){
    return right2;
  }
}