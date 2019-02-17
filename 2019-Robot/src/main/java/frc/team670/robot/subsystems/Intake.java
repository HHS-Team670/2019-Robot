/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the Intake mechanism of the robot.
 */
public class Intake extends BaseIntake {

  private static final int ROLLER_CONTINUOUS_CURRENT = 30, ROLLER_PEAK_CURRENT = 0;

  public static final int INTAKE_ANGLE_IN = -90, INTAKE_ANGLE_DEPLOYED = 90;
  public static final double INTAKE_FIXED_LENGTH_IN_INCHES = 11.25, INTAKE_ROTATING_LENGTH_IN_INCHES = 14;
  private static final double MAX_BASE_OUTPUT = 0.75;
  private static final double kF = 0, kP = 0.35, kI = 0, kD = 0;

  // Motion Magic
  private static final int kPIDLoopIdx = 0, MOTION_MAGIC_SLOT = 0, kTimeoutMs = 0;
  private static final int OFFSET_FROM_ENCODER_ZERO = -260;
  private static final int FORWARD_SOFT_LIMIT = 850, REVERSE_SOFT_LIMIT = -940;
  private static final int CONTINUOUS_CURRENT_LIMIT = 20, PEAK_CURRENT_LIMIT = 0;
  private final static int INTAKE_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 120,  INTAKE_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_SECOND = 400;
  private static final int QUAD_ENCODER_MAX = FORWARD_SOFT_LIMIT + 200, QUAD_ENCODER_MIN = REVERSE_SOFT_LIMIT - 200;

  private static final double ARBITRARY_FEED_FORWARD = 0.175;

  private static final int TICKS_PER_ROTATION = 4096;

  private TalonSRX roller;
  
  private Point2D.Double intakeCoord;

  public Intake() {
    super(new TalonSRX(RobotMap.INTAKE_BASE_TALON), ARBITRARY_FEED_FORWARD, FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT, true, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, OFFSET_FROM_ENCODER_ZERO);
    
    roller = new TalonSRX(RobotMap.INTAKE_ROLLER_TALON);

    roller.setInverted(true);
    roller.setNeutralMode(NeutralMode.Coast);

    intakeCoord = new Point2D.Double();

    rotator.setInverted(true);
    rotator.setSensorPhase(false); // Positive is inwards movement, negative is outward

    if(rotatorSensorCollection.isRevLimitSwitchClosed()) {
      rotator.setSelectedSensorPosition(REVERSE_SOFT_LIMIT);
    }
    else if(rotatorSensorCollection.isFwdLimitSwitchClosed()) {
      rotator.setSelectedSensorPosition(FORWARD_SOFT_LIMIT);
    }

    stop();
    setMotionMagicPIDValues();
  }

  // May need to set tolerance
  private void setMotionMagicPIDValues() {
    rotator.selectProfileSlot(MOTION_MAGIC_SLOT, kPIDLoopIdx);
    rotator.config_kF(MOTION_MAGIC_SLOT, kF, kTimeoutMs);
    rotator.config_kP(MOTION_MAGIC_SLOT, kP, kTimeoutMs);
    rotator.config_kI(MOTION_MAGIC_SLOT, kI, kTimeoutMs);
    rotator.config_kD(MOTION_MAGIC_SLOT, kD, kTimeoutMs);
    rotator.configMotionCruiseVelocity(INTAKE_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    rotator.configMotionAcceleration(INTAKE_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_SECOND, kTimeoutMs);

    rotator.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    rotator.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    rotator.configPeakOutputForward(MAX_BASE_OUTPUT, RobotConstants.kTimeoutMs);
    rotator.configPeakOutputReverse(-MAX_BASE_OUTPUT, RobotConstants.kTimeoutMs);

    rotator.setNeutralMode(NeutralMode.Brake);
    roller.setNeutralMode(NeutralMode.Coast);
    roller.configContinuousCurrentLimit(ROLLER_CONTINUOUS_CURRENT);
    roller.configPeakCurrentLimit(ROLLER_PEAK_CURRENT);
    roller.enableCurrentLimit(true);
  }

  /**
   * Should set the setpoint for the Motion Magic on the intake
   */
  public void setMotionMagicSetpointAngle(double intakeAngle) {
    setpoint = convertIntakeDegreesToTicks(intakeAngle);
    SmartDashboard.putNumber("MotionMagicSetpoint", setpoint);
    rotator.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Should return the setpoint for the motion magic on the base motor
   */
  public Point2D.Double getMotionMagicDestinationCoordinates(){
    double x = INTAKE_ROTATING_LENGTH_IN_INCHES * Math.cos(Math.toRadians(getMotionMagicSetpoint()));
    double y = INTAKE_FIXED_LENGTH_IN_INCHES + INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(getMotionMagicSetpoint());
    return new Point2D.Double(x, y);
  }

  public void setRotatorNeutralMode(NeutralMode mode) {
    rotator.setNeutralMode(mode);
  }
  /**
   * Returns the x, y coordinates of the top of the intake
   */
  public Point2D.Double getIntakeCoordinates(){
    double x = INTAKE_ROTATING_LENGTH_IN_INCHES * Math.cos(getAngleInDegrees());
    double y = INTAKE_FIXED_LENGTH_IN_INCHES + INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(getAngleInDegrees());
    intakeCoord.setLocation(x, y);
    return intakeCoord;
  }

  /**
   * Runs the intake at a given percent power
   * 
   * @param percentOutput The desired percent power for the rollers to run at [-1,
   *                      1]
   */
  public void runIntake(double power, boolean runningIn) {
    power *= runningIn ? 1 : -1;
    roller.set(ControlMode.PercentOutput, power);
  }
  /**
   * Converts an intake angle into ticks
   */
  private static int convertIntakeDegreesToTicks(double degrees) {
    //If straight up is 0 and going forward is positive
    // percentage * half rotation
    return (int)((degrees / 360) * TICKS_PER_ROTATION);
  }

  /**
   * Converts intake ticks into an angle
   */
  private static double convertIntakeTicksToDegrees(double ticks) {
    //If straight up is 0 and going forward is positive
    return ((360 * ticks) / TICKS_PER_ROTATION);
  }

  @Override
  public double getArbitraryFeedForwardAngleMultiplier() {
    double angleInDegrees = getAngleInDegrees();
    return -1 * Math.sin(Math.toRadians(angleInDegrees));
    // System.out.println("IntakeArbitraryFeedforward: " + output);
  }

  public void sendDataToDashboard() {
    SmartDashboard.putNumber("Unadjusted Absolute Ticks", getUnadjustedPulseWidth());
    SmartDashboard.putNumber("Absolute Ticks", getRotatorPulseWidth());
    SmartDashboard.putNumber("Quadrature Ticks", getPositionTicks());
  }

  @Override
  public void initDefaultCommand() {

  }

  @Override
  public double getAngleInDegrees() {
    return convertIntakeTicksToDegrees(getPositionTicks());
  }

}
