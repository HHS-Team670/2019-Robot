/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team670.robot.commands.drive.teleop.XboxRocketLeagueDrive;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.MustangDriveBaseEncoder;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Represents a tank drive base.
 * 
 * @author shaylandias
 */
public class DriveBase extends Subsystem {

  private static final int VELOCITY_PID_SLOT = 1, ENCODERS_PID_SLOT = 2;

  private CANSparkMax left1, left2, right1, right2;
  private SpeedControllerGroup left, right;
  private DifferentialDrive driveTrain;
  private List<CANSparkMax> leftControllers, rightControllers;
  private List<CANSparkMax> allMotors;
  private MustangDriveBaseEncoder leftMustangEncoder, rightMustangEncoder;
  private Encoder leftDIOEncoder, rightDIOEncoder;


  private final double P_P = 0.1, P_I = 1E-4, P_D = 1, P_FF = 0; // Position PID Values. Set based off the default in
                                                                 // REV Robotics example code.
  private final double V_P = 5E-5, V_I = 1E-5, V_D = 0, V_FF = 0; // Velocity PID Values. Set based off the default in
                                                                  // REV Robotics example code.

  public DriveBase() {
    left1 = new CANSparkMax(RobotMap.SPARK_LEFT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    left2 = new CANSparkMax(RobotMap.SPARK_LEFT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    right1 = new CANSparkMax(RobotMap.SPARK_RIGHT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    right2 = new CANSparkMax(RobotMap.SPARK_RIGHT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);

    allMotors = new ArrayList<CANSparkMax>();
    leftControllers = Arrays.asList(left1, left2);
    rightControllers = Arrays.asList(right1, right2);
    allMotors.addAll(leftControllers);
    allMotors.addAll(rightControllers);

    setMotorsInvert(leftControllers, false);

    // Invert this so it will work properly with the CANPIDController
    // and then invert the SpeedController to compensate for automatic inversion.
    setMotorsInvert(rightControllers, true);

    left2.follow(left1);
    right2.follow(right1);

    left = new SpeedControllerGroup(left1, left2);
    right = new SpeedControllerGroup(right1, right2);
    // The DifferentialDrive inverts the right side automatically, however we want
    // to invert straight from the Spark so that we can
    // still use it properly with the CANPIDController, so we need to reverse the
    // automatic invert.
    right.setInverted(true);

    driveTrain = new DifferentialDrive(left, right);
    driveTrain.setMaxOutput(1.0);

    // Set PID Values
    left1.getPIDController().setP(P_P, ENCODERS_PID_SLOT);
    left1.getPIDController().setI(P_I, ENCODERS_PID_SLOT);
    left1.getPIDController().setD(P_D, ENCODERS_PID_SLOT);
    left1.getPIDController().setFF(P_FF, ENCODERS_PID_SLOT);
    left1.getPIDController().setOutputRange(-1, 1);

    right1.getPIDController().setP(V_P, VELOCITY_PID_SLOT);
    right1.getPIDController().setI(V_I, VELOCITY_PID_SLOT);
    right1.getPIDController().setD(V_D, VELOCITY_PID_SLOT);
    right1.getPIDController().setFF(V_FF, VELOCITY_PID_SLOT);

    setRampRate(allMotors, 0.254); // Will automatically cook some Cheezy Poofs

    // DIO Encoders
    try {
      leftDIOEncoder = new Encoder(RobotMap.LEFT_ENCODER_CHANNEL_A, RobotMap.LEFT_ENCODER_CHANNEL_B, false, EncodingType.k4X);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error Instantiating leftDIOEncoder: " + ex.getMessage(), true);
      leftDIOEncoder = null;
    }
    
    try {
      rightDIOEncoder = new Encoder(RobotMap.RIGHT_ENCODER_CHANNEL_A, RobotMap.RIGHT_ENCODER_CHANNEL_B, false, EncodingType.k4X);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error Instantiating rightDIOEncoder: " + ex.getMessage(), true);
      rightDIOEncoder = null;
    }

    double distancePerPulse = (1 / RobotConstants.DIO_TICKS_PER_ROTATION)
        * (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);

    if (leftDIOEncoder != null){
      leftDIOEncoder.setDistancePerPulse(distancePerPulse);
      leftDIOEncoder.setReverseDirection(true);
    }
    if(rightDIOEncoder != null) {
      rightDIOEncoder.setDistancePerPulse(distancePerPulse);
      rightDIOEncoder.setReverseDirection(false);
    }

    leftMustangEncoder = new MustangDriveBaseEncoder(leftDIOEncoder, left1.getEncoder());
    rightMustangEncoder = new MustangDriveBaseEncoder(rightDIOEncoder, right1.getEncoder());

  }

  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton).
   * Squares inputs to linearize them.
   * 
   * @param leftSpeed  Speed for left side of drive base [-1, 1]. Automatically
   *                   squares this value to linearize it.
   * @param rightSpeed Speed for right side of drive base [-1, 1]. Automatically
   *                   squares this value to linearize it.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  public void initAutonDrive() {
    setMotorsBrakeMode(allMotors, IdleMode.kBrake);
  }

  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton)
   * 
   * @param leftSpeed     Speed for left side of drive base [-1, 1]
   * @param rightSpeed    Speed for right side of drive base [-1, 1]
   * @param squaredInputs If true, decreases sensitivity at lower inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
    driveTrain.tankDrive(leftSpeed, rightSpeed, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using a curvature drive configuration (wheel)
   * 
   * @param xSpeed      The forward throttle speed [-1, 1]
   * @param zRotation   The amount of rotation to turn [-1, 1] with positive being
   *                    right
   * @param isQuickTurn If true enables turning in place and running one side
   *                    backwards to turn faster
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with
   * twist)
   * 
   * @param xSpeed      The forward throttle speed [-1, 1]
   * @param zRotation   The amount of rotation to turn [-1, 1] with positive being
   *                    right
   * @param isQuickTurn If true, decreases sensitivity at lower inputs
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
    driveTrain.arcadeDrive(xSpeed, zRotation, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with
   * twist)
   * 
   * @param xSpeed    The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being
   *                  right
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
  public double getRobotInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of all the motor controllers on the robot
   */
  public double getRobotOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput() + right1.getAppliedOutput()
        + right2.getAppliedOutput();
    return output;
  }

  /**
   * Return the left CANEncoder Object. Do not access this for PID Controllers
   * anymore, use the internal PIDControllers for the SparkMAX motors.
   */
  public CANEncoder getLeftSparkEncoder() {
    return left1.getEncoder();

  }

  /**
   * Return the right CanEncoder Object. Do not access this for PID Controllers
   * anymore, use the internal PIDControllers for the SparkMAX motors.
   */
  public CANEncoder getRightSparkEncoder() {
    return right1.getEncoder();
  }

  /**
   * Sets the PIDControllers setpoints for the left and right side motors to the
   * given positions in ticks forward.
   * 
   * @param deltaLeft  The desired change in left position in encoder ticks
   * @param deltaRight The desired change in right position in encoder ticks
   */
  public void setSparkEncodersControl(double deltaLeft, double deltaRight) {
    left1.getPIDController().setReference(getLeftSparkEncoderPosition() + deltaLeft, ControlType.kPosition,
        ENCODERS_PID_SLOT);
    right1.getPIDController().setReference(getRightSparkEncoderPosition() + deltaRight, ControlType.kPosition,
        ENCODERS_PID_SLOT);
  }

  /**
   * Sets the velocities of the left and right motors of the robot.
   * 
   * @param leftVel  Velocity for left motors in inches/sec
   * @param rightVel Velocity for right motors in inches/sec
   */
  public void setSparkVelocityControl(double leftVel, double rightVel) {
    leftVel = MathUtils
        .convertInchesPerSecondToDriveBaseRoundsPerMinute(MathUtils.convertInchesToDriveBaseTicks(leftVel));
    rightVel = MathUtils
        .convertInchesPerSecondToDriveBaseRoundsPerMinute(MathUtils.convertInchesToDriveBaseTicks(rightVel));
    left1.getPIDController().setReference(leftVel, ControlType.kVelocity, VELOCITY_PID_SLOT);
    right1.getPIDController().setReference(rightVel, ControlType.kVelocity, VELOCITY_PID_SLOT);
  }

  /**
   * Gets the encoder position of the front left motor in ticks.
   */
  public int getLeftSparkEncoderPosition() {
    return (int) (left1.getEncoder().getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
  }

  /**
   * Gets the encoder position of the front right motor in ticks.
   */
  public int getRightSparkEncoderPosition() {
    return (int) (right1.getEncoder().getPosition() / RobotConstants.SPARK_TICKS_PER_ROTATION);
  }

  /**
   * Returns the velocity of the left Spark controller in RPM
   */
  public int getLeftSparkVelocityInRotationsPerMinute() {
    return (int) left1.getEncoder().getVelocity();
  }

  /**
   * Returns the velocity of the right Spark controller in RPM
   */
  public int getRightSparkVelocityInRotationsPerMinute() {
    return (int) right1.getEncoder().getVelocity();
  }

  /**
   * Inverts a list of motors.
   */
  private void setMotorsInvert(List<CANSparkMax> motorGroup, boolean invert) {
    for (CANSparkMax m : motorGroup) {
      m.setInverted(invert);
    }
  }

  /**
   * Sets array of motors to be brushless
   */

  private void setMotorsBrushless(List<CANSparkMax> motorGroup) {
    for (CANSparkMax m : motorGroup) {
      m.setMotorType(CANSparkMaxLowLevel.MotorType.kBrushless);
    }
  }

  /**
   * Sets array of motors to be of a specified mode
   */
  public void setMotorsNeutralMode(IdleMode mode) {
    for (CANSparkMax m : allMotors) {
      m.setIdleMode(mode);
    }
  }

  /**
   * Sets array of motor to coast mode
   */
  public void setMotorsCoastMode(List<CANSparkMax> motorGroup, IdleMode mode) {
    for (CANSparkMax m : motorGroup) {
      m.setIdleMode(IdleMode.kCoast);
    }
  }

  /**
   * Sets array of motor to brake mode
   */
  public void setMotorsBrakeMode(List<CANSparkMax> motorGroup, IdleMode mode) {
    for (CANSparkMax m : motorGroup) {
      m.setIdleMode(IdleMode.kBrake);
    }
  }

  /*
   * Gets the voltage fed into the motor controllers on the left side of the robot
   */
  public double getLeftInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage();
    return output;
  }

  /*
   * Get the voltage fed into the motor controllers on the right side of the robot
   */
  public double getRightInputVoltage() {
    double output = right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of the motor controllers on the left side of the
   * robot
   */
  public double getLeftOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output voltage of the motor controllers on the right side of the
   * robot
   */
  public double getRightOutputVoltage() {
    double output = right1.getAppliedOutput() + right2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the left side
   * of the robot
   */
  public double getLeftOutputCurrent() {
    double output = left1.getOutputCurrent() + left2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the output current (in amps) of the motor controllers on the right side
   * of the robot
   */
  public double getRightOutputCurrent() {
    double output = right1.getOutputCurrent() + right2.getOutputCurrent();
    return output;
  }

  /*
   * Gets the input voltage of all the drivebase motor controllers on the robot
   */
  public double getDriveBaseInputVoltage() {
    double output = left1.getBusVoltage() + left2.getBusVoltage() + right1.getBusVoltage() + right2.getBusVoltage();
    return output;
  }

  /*
   * Gets the output voltage of all the drivebase motor controllers on the robot
   */
  public double getDriveBaseOutputVoltage() {
    double output = left1.getAppliedOutput() + left2.getAppliedOutput() + right1.getAppliedOutput()
        + right2.getAppliedOutput();
    return output;
  }

  /*
   * Gets the output current of all the motor controllers on the robot
   */
  public double getRobotOutputCurrent() {
    double output = left1.getOutputCurrent() + left2.getOutputCurrent() + right1.getOutputCurrent()
        + right2.getOutputCurrent();
    return output;
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new XboxRocketLeagueDrive());
  }

  public Encoder getLeftDIOEncoder() {
    return leftDIOEncoder;
  }

  public Encoder getRightDIOEncoder() {
    return rightDIOEncoder;
  }

  /**
   * Gets the tick count of the left encoder
   */
  public int getLeftDIOEncoderPosition() {
    return leftDIOEncoder.get();
  }


  /**
   * Returns the velocity of the left side of the drivebase in inches/second from
   * the DIO Encoder
   */
  public double getLeftDIOEncoderVelocityInches() {
    return leftDIOEncoder.getRate();
  }

   /**
   * Returns the velocity of the left side of the drivebase in ticks/second from
   * the DIO Encoder
   */
  public double getLeftDIOEncoderVelocityTicks() {
    return MathUtils.convertInchesToDriveBaseTicks(leftDIOEncoder.getRate());
  }

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the DIO Encoder
   */
  public double getRightDIOEncoderVelocityInches() {
    return rightDIOEncoder.getRate();
  }

  /**
   * Returns the velocity of the right side of the drivebase in ticks/second from
   * the DIO Encoder
   */
  public double getRightDIOEncoderVelocityTicks() {
    return MathUtils.convertInchesToDriveBaseTicks(rightDIOEncoder.getRate());
  }

  /**
   * Returns the velocity of the left side of the drivebase in inches/second from
   * the Spark Encoder
   */
  public double getLeftSparkEncoderVelocityInches() {
    return (MathUtils.convertDriveBaseTicksToInches(left1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
  }

   /**
   * Returns the velocity of the left side of the drivebase in ticks/second from
   * the Spark Encoder
   */
  public double getLeftSparkEncoderVelocityTicks() {
    return (left1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
  }

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the Spark Encoder
   */
  public double getRightSparkEncoderVelocityInches() {
    return (MathUtils.convertDriveBaseTicksToInches(right1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
  }

  /**
   * Returns the velocity of the right side of the drivebase in ticks/second from
   * the Spark Encoder
   */
  public double getRightSparkEncoderVelocityTicks() {
    return (right1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
  }


  public double getLeftDIODistanceInches() {
    return leftDIOEncoder.getDistance();
  }

  public double getRightDIODistanceInches() {
    return rightDIOEncoder.getDistance();
  }

  /**
   * Gets the tick count of the right encoder
   */
  public int getRightDIOEncoderPosition() {
    return rightDIOEncoder.get();
  }

  public List<CANSparkMax> getLeftControllers() {
    return leftControllers;
  }

  public List<CANSparkMax> getRightControllers() {
    return rightControllers;
  }

  /**
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setRampRate(List<CANSparkMax> motors, double rampRate) {
    for (CANSparkMax m : motors) {
      m.setRampRate(rampRate);
    }
  }

  /**
   * Returns the MustangDriveBaseEncoder used for the left motors
   */
  public MustangDriveBaseEncoder getLeftMustangDriveBaseEncoder(){
    return leftMustangEncoder;
  }

  /**
   * Returns the MustangDriveBaseEncoder used for the right motors
   */
  public MustangDriveBaseEncoder getRightMustangDriveBaseEncoder(){
    return rightMustangEncoder;
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the left motors in ticks
   */
  public int getLeftMustangEncoderPositionInTicks(){
    return leftMustangEncoder.getPositionTicks();
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the right motors in ticks
   */
  public int getRightMustangEncoderPositionInTicks(){
    return rightMustangEncoder.getPositionTicks();
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the left motors in inches
   */
  public double getLeftMustangEncoderPositionInInches(){
    return leftMustangEncoder.getPositionInches();
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the right motors in inches
   */
  public double getRightMustangEncoderPositionInInches(){
    return rightMustangEncoder.getPositionInches();
  }
    /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the left motors in ticks/second
   */
  public double getLeftMustangEncoderVelocityInTicksPerSecond(){
    return leftMustangEncoder.getVelocityTicks();
  }

  /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the right motors in ticks/second
   */
  public double getRightMustangEncoderVelocityInTicksPerSecond(){
    return rightMustangEncoder.getVelocityTicks();
  }

  /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the left motors in inches/second
   */
  public double getLeftMustangEncoderVelocityInInchesPerSecond(){
    return leftMustangEncoder.getVelocityInches();
  }

  /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the right motors in inches/second
   */
  public double getRightMustangEncoderVelocityInInchesPerSecond(){
    return rightMustangEncoder.getVelocityInches();
  }
}