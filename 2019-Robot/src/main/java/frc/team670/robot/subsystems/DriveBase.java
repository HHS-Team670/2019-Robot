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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.drive.teleop.XboxRocketLeagueDrive;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.sensors.MustangDriveBaseEncoder;
import frc.team670.robot.dataCollection.sensors.NavX;
import frc.team670.robot.utils.Logger;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * Represents a tank drive base.
 * 
 * @author shaylandias
 */
public class DriveBase extends Subsystem {

  private static final int VELOCITY_PID_SLOT = 1, ENCODERS_PID_SLOT = 2;

  private CANSparkMax left1, left2, right1, right2;
  private CANEncoder leftEncoder, rightEncoder;
  private SpeedControllerGroup left, right;
  private DifferentialDrive driveTrain;
  private List<CANSparkMax> leftControllers, rightControllers;
  private List<CANSparkMax> allMotors;
  private MustangDriveBaseEncoder leftMustangEncoder, rightMustangEncoder;
  private Encoder leftDIOEncoder, rightDIOEncoder;
  private DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(RobotConstants.kDriveKinematics);
  private NavX navXMicro;

  private static final double drivebaseGearRatio = 8.45;

  private final double P_P = 0.1, P_I = 1E-4, P_D = 1, P_FF = 0; // Position PID Values. Set based off the default in
                                                                 // REV Robotics example code.
  private final double V_P = 10, V_I = 1E-6, V_D = 0, V_FF = 0; // Velocity PID Values. Set based off the default in
                                                                // REV Robotics example code.

  public DriveBase() {
    left1 = new CANSparkMax(RobotMap.SPARK_LEFT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    left2 = new CANSparkMax(RobotMap.SPARK_LEFT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);
    right1 = new CANSparkMax(RobotMap.SPARK_RIGHT_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
    right2 = new CANSparkMax(RobotMap.SPARK_RIGHT_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);

    left1.restoreFactoryDefaults();
    left2.restoreFactoryDefaults();
    right1.restoreFactoryDefaults();
    right2.restoreFactoryDefaults();


    leftEncoder = left1.getEncoder();
    rightEncoder = right1.getEncoder();

    LogLeftMotorInfo();
    LogRightMotorInfo();
    
    double sparkMaxVelocityConversionFactor = RobotConstants.DRIVEBASE_METERS_PER_ROTATION / 60;//(double)RobotConstants.SPARK_TICKS_PER_ROTATION;
    left1.getEncoder().setVelocityConversionFactor(sparkMaxVelocityConversionFactor);
    right1.getEncoder().setVelocityConversionFactor(sparkMaxVelocityConversionFactor); //Do not invert for right side
    left2.getEncoder().setVelocityConversionFactor(sparkMaxVelocityConversionFactor);
    right2.getEncoder().setVelocityConversionFactor(sparkMaxVelocityConversionFactor);
    // these are commented cuuz libraruy being used is
    allMotors = new ArrayList<CANSparkMax>();
    leftControllers = Arrays.asList(left1, left2);
    rightControllers = Arrays.asList(right1, right2);
    allMotors.addAll(leftControllers);
    allMotors.addAll(rightControllers);

    initBrakeMode();

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

    left1.getPIDController().setP(V_P, VELOCITY_PID_SLOT);
    left1.getPIDController().setI(V_I, VELOCITY_PID_SLOT);
    left1.getPIDController().setD(V_D, VELOCITY_PID_SLOT);
    left1.getPIDController().setFF(V_FF, VELOCITY_PID_SLOT);

    right1.getPIDController().setP(V_P, VELOCITY_PID_SLOT);
    right1.getPIDController().setI(V_I, VELOCITY_PID_SLOT);
    right1.getPIDController().setD(V_D, VELOCITY_PID_SLOT);
    right1.getPIDController().setFF(V_FF, VELOCITY_PID_SLOT);

    right1.getPIDController().setP(P_P, ENCODERS_PID_SLOT);
    right1.getPIDController().setI(P_I, ENCODERS_PID_SLOT);
    right1.getPIDController().setD(P_D, ENCODERS_PID_SLOT);
    right1.getPIDController().setFF(P_FF, ENCODERS_PID_SLOT);
    right1.getPIDController().setOutputRange(-1, 1);

    setRampRate(allMotors, 0.36); // Will automatically cook some Cheezy Poofs

    // DIO Encoders
    try {
      leftDIOEncoder = new Encoder(RobotMap.LEFT_ENCODER_CHANNEL_A, RobotMap.LEFT_ENCODER_CHANNEL_B, false,
          EncodingType.k4X);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error Instantiating leftDIOEncoder: " + ex.getMessage(), true);
      leftDIOEncoder = null;
    }

    try {
      // rightDIOEncoder = new Encoder(RobotMap.RIGHT_ENCODER_CHANNEL_A, RobotMap.RIGHT_ENCODER_CHANNEL_B, false,
          // EncodingType.k4X);
      rightDIOEncoder = null;
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error Instantiating rightDIOEncoder: " + ex.getMessage(), true);
      rightDIOEncoder = null;
    }

    double distancePerPulse = (1 / RobotConstants.DIO_TICKS_PER_ROTATION)
        * (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);

    if (leftDIOEncoder != null) {
      leftDIOEncoder.setDistancePerPulse(distancePerPulse);
      leftDIOEncoder.setReverseDirection(true);
    }
    if (rightDIOEncoder != null) {
      // rightDIOEncoder.setDistancePerPulse(distancePerPulse);
      // rightDIOEncoder.setReverseDirection(true);
    }

    leftMustangEncoder = new MustangDriveBaseEncoder(null, left1.getEncoder(), false);
    rightMustangEncoder = new MustangDriveBaseEncoder(null, right1.getEncoder(), true);
    navXMicro = new NavX(RobotMap.NAVX_PORT);

  }

  public void feedWatchDog(){
    driveTrain.feed();
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

  // https://github.com/wpilibsuite/allwpilib/blob/master/wpilibj/src/main/java/edu/wpi/first/wpilibj/SpeedController.java#L32
  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    tankDrive(leftVoltage / RobotController.getBatteryVoltage(), rightVoltage / RobotController.getBatteryVoltage());
  }

  public void initBrakeMode() {
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


  public CANEncoder getLeftEncoder() {
    return left1.getEncoder();
  }

  public CANEncoder getRightEncoder() {
    return right1.getEncoder();
  }

  /**
   * Returns the left DIO Encoder
   */
  public Encoder getLeftDIOEncoder() {
    return leftDIOEncoder;
  }

  /**
   * Returns the right DIO Encoder
   */
  public Encoder getRightDIOEncoder() {
    return rightDIOEncoder;
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
    leftVel = convertInchesPerSecondToDriveBaseRoundsPerMinute(convertInchesToDriveBaseTicks(leftVel));
    rightVel = convertInchesPerSecondToDriveBaseRoundsPerMinute(convertInchesToDriveBaseTicks(rightVel));
    // System.out.println("LeftVel: " + leftVel);
    // System.out.println("RightVel: " + rightVel);
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
   * Gets the encoder position of the front left motor in ticks.
   */
  public int getLeftDIOEncoderPosition() {
    return leftDIOEncoder.get();
  }

  /**
   * Gets the tick count of the right encoder
   */
  public int getRightDIOEncoderPosition() {
    return rightDIOEncoder.get();
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

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the Spark Encoder
   */
  // public double getLeftSparkEncoderVelocityInches() {
  // return
  // (DriveBase.convertDriveBaseTicksToInches(left1.getEncoder().getVelocity() /
  // RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
  // }

  /**
   * Returns the velocity of the right side of the drivebase in ticks/second from
   * the Spark Encoder
   */
  public double getLeftSparkEncoderVelocityTicks() {
    return (left1.getEncoder().getVelocity() / RobotConstants.SPARK_TICKS_PER_ROTATION / 60);
  }

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the Spark Encoder
   */
  // public double getRightSparkEncoderVelocityInches() {
  // return
  // (DriveBase.convertDriveBaseTicksToInches(right1.getEncoder().getVelocity() /
  // RobotConstants.SPARK_TICKS_PER_ROTATION) / 60);
  // }

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
      m.setClosedLoopRampRate(rampRate);
      m.setOpenLoopRampRate(rampRate);
      //m.setRampRate(rampRate);
    }
  }

  /**
   * Converts a tick value taken from a drive base DIO encoder to inches.
   */
  public static double convertDriveBaseTicksToInches(double ticks) {
    double rotations = ticks / RobotConstants.DIO_TICKS_PER_ROTATION;
    return rotations * Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER;
  }

  public static double convertSparkRevolutionsToInches(double revolutions) {
    // rev * 2piR in / rev
    return revolutions * Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER / drivebaseGearRatio;
  }

  /**
   * Converts an inch value into drive base DIO Encoder ticks.
   */
  public static int convertInchesToDriveBaseTicks(double inches) {
    double rotations = inches / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
    return (int) (rotations * RobotConstants.DIO_TICKS_PER_ROTATION);
  }

  /**
   * Gets inches per rotations of a NEO motor on the drive base since SparkMAX
   * encoders work in rotations.
   */
  public static double convertDriveBaseRotationsToInches(double rotations) {
    return RobotConstants.DRIVEBASE_INCHES_PER_ROTATION * rotations;
  }

  /**
   * Gets rotations of a NEO motor on the drive base per a value in inches ince
   * SparkMAX encoders work in rotations.
   */
  public static double convertInchesToDriveBaseRotations(double inches) {
    return inches / RobotConstants.DRIVEBASE_INCHES_PER_ROTATION;
  }

  /**
   * Converts a value of per second of the DriveBase Rounds Per Minute
   */
  public static double convertInchesPerSecondToDriveBaseRoundsPerMinute(double inchesPerSecond) {
    // (Inches/seconds) * (60 seconds/1 minute) * ((2 * Diameter inches)/Rotation)
    return inchesPerSecond * 60 / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
  }

  /**
   * Returns the MustangDriveBaseEncoder used for the left motors
   */
  public MustangDriveBaseEncoder getLeftMustangDriveBaseEncoder() {
    return leftMustangEncoder;
  }

  /**
   * Returns the MustangDriveBaseEncoder used for the right motors
   */
  public MustangDriveBaseEncoder getRightMustangDriveBaseEncoder() {
    return rightMustangEncoder;
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the left motors
   * in ticks
   */
  public int getLeftMustangEncoderPositionInTicks() {
    return leftMustangEncoder.getPositionTicks();
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the right motors
   * in ticks
   */
  public int getRightMustangEncoderPositionInTicks() {
    return rightMustangEncoder.getPositionTicks();
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the left motors
   * in inches
   */
  public double getLeftMustangEncoderPositionInInches() {
    return leftMustangEncoder.getPositionInches();
  }

  /**
   * Returns the position of the MustangDriveBaseEncoder used for the right motors
   * in inches
   */
  public double getRightMustangEncoderPositionInInches() {
    return rightMustangEncoder.getPositionInches();
  }

  /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the left motors
   * in ticks/second
   */
  public double getLeftMustangEncoderVelocityInTicksPerSecond() {
    return leftMustangEncoder.getVelocityTicks();
  }

  public void initCoastMode() {
    setMotorsNeutralMode(IdleMode.kCoast);
  }

  /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the right motors
   * in ticks/second
   */
  public double getRightMustangEncoderVelocityInTicksPerSecond() {
    return rightMustangEncoder.getVelocityTicks();
  }

  /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the left motors
   * in inches/second
   */
  public double getLeftMustangEncoderVelocityInInchesPerSecond() {
    return leftMustangEncoder.getVelocityInches();
  }

  /**
   * Returns the velocity of the MustangDriveBaseEncoder used for the right motors
   * in inches/second
   */
  public double getRightMustangEncoderVelocityInInchesPerSecond() {
    return rightMustangEncoder.getVelocityInches();
  }

  public void sendEncoderDataToDashboard() {
    // if (leftDIOEncoder != null) {
    //   SmartDashboard.putNumber("Left DIO Encoder: ", leftMustangEncoder.getPositionInches());
    // }

    // if (rightDIOEncoder != null) {
    //   SmartDashboard.putNumber("Right Encoder: ", rightDIOEncoder.get());
    // }

    // if (leftDIOEncoder == null) {
    //   SmartDashboard.putString("Left DIO Encoder:", "LEFT DIO ENCODER IS NULL!");
    // }
    // if (rightDIOEncoder == null) {
    //   SmartDashboard.putNumber("Right Encoder:", rightMustangEncoder.getPositionInches());
    // }
    if(leftMustangEncoder != null) {
      SmartDashboard.putString("Left Encoder Inches", "0"); //leftMustangEncoder.getPositionInches() + "");
    } else {
      SmartDashboard.putString("Left Encoder Inches", "0"); //"null");
    }
    if(rightMustangEncoder != null) {
      SmartDashboard.putString("Right Encoder Inches", "0"); //rightMustangEncoder.getPositionInches() + "");
    } else {
      SmartDashboard.putString("Left Encoder Inches", "0"); //"null");
    }
    SmartDashboard.putString("Left M Position Ticks", "0"); leftEncoder.getPosition(); //+ "");
    SmartDashboard.putString("Left M Velocity Ticks", "0"); //left1.getEncoder().getVelocity() + "");
    SmartDashboard.putString("Left S Position Ticks", "0"); //left2.getEncoder().getPosition() + "");
    SmartDashboard.putString("Left S Velocity Ticks", "0"); //left2.getEncoder().getVelocity() + "");
    SmartDashboard.putString("Right M Position Ticks", "0"); //right1.getEncoder().getPosition() + "");
    SmartDashboard.putString("Right M Velocity Ticks", "0"); //right1.getEncoder().getVelocity() + "");
    SmartDashboard.putString("Right S Position Ticks", "0"); //right2.getEncoder().getPosition() + "");
    SmartDashboard.putString("Right S Velocity Ticks", "0"); //right2.getEncoder().getVelocity() + "");
    leftEncoder.getPosition();
    leftEncoder.getVelocity();
    rightEncoder.getPosition();
    rightEncoder.getVelocity();
  }

    
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
   // Logger.consoleLog("LeftEncoderVelocity: %s, RightEncoderVelocity: %s", getLeftEncoder().getVelocity(), getRightEncoder().getVelocity());
    m_odometry.update(Rotation2d.fromDegrees(getHeading()),
                                  new DifferentialDriveWheelSpeeds(
                                      leftEncoder.getVelocity(),
                                      rightEncoder.getVelocity()
                                  ));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose);
  }

  public void resetOdometry(){
    m_odometry = new DifferentialDriveOdometry(RobotConstants.kDriveKinematics);
    zeroHeading();
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }
  
  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navXMicro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navXMicro.getAngle(), 360) * (RobotConstants.kNavXReversed ? -1. : 1.);
  }

  public void LogLeftMotorInfo(){
    Logger.consoleLog("ClosedLoopRampRate1 %s, OpenLoopRampRate1 %s", left1.getClosedLoopRampRate(), left1.getOpenLoopRampRate());
    Logger.consoleLog("ClosedLoopRampRate2 %s, OpenLoopRampRate2 %s", left2.getClosedLoopRampRate(), left2.getClosedLoopRampRate());
  }

  public void LogRightMotorInfo(){
    Logger.consoleLog("ClosedLoopRampRate1 %s, OpenLoopRampRate1 %s", right1.getClosedLoopRampRate(), right1.getOpenLoopRampRate());
    Logger.consoleLog("ClosedLoopRampRate2 %s, OpenLoopRampRate2 %s", right2.getClosedLoopRampRate(), right2.getClosedLoopRampRate());
  }



}