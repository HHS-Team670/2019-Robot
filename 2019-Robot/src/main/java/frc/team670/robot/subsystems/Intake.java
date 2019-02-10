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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;

/**
 * Represents the Intake mechanism of the robot.
 */
public class Intake extends BaseIntake {

  public static final int INTAKE_ANGLE_IN = -90, INTAKE_ANGLE_DEPLOYED = 90;
  public static final double INTAKE_FIXED_LENGTH_IN_INCHES = 0, INTAKE_ROTATING_LENGTH_IN_INCHES = 0; //TODO set actual value
  private static final double MAX_BASE_OUTPUT = 0.75;
  private static final double kF = 0, kP = 0.3, kI = 0, kD = 0; //TODO figure out what these are
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0; //TODO Set this
  private static final int FORWARD_SOFT_LIMIT = 971, REVERSE_SOFT_LIMIT = -910; // Negative is Forward, positive is backwards
  private static final double RAMP_RATE = 0.1;
  private static final int OFFSET_FROM_ENCODER_ZERO = 670;
  private static final int CONTINUOUS_CURRENT_LIMIT = 20, PEAK_CURRENT_LIMIT = 0;
  private final static int INTAKE_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 120,  INTAKE_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_SECOND = 400;
  private static final int QUAD_ENCODER_MIN = FORWARD_SOFT_LIMIT + 200, QUAD_ENCODER_MAX = REVERSE_SOFT_LIMIT - 200;

  private static final double ARBITRARY_FEED_FORWARD = 0.15;

  private VictorSPX rollerVictor;
  
  private Point2D.Double intakeCoord;

  public Intake() {
    super(new TalonSRX(RobotMap.INTAKE_BASE_TALON), ARBITRARY_FEED_FORWARD, FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT, true, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, OFFSET_FROM_ENCODER_ZERO);
    
    rollerVictor = new VictorSPX(RobotMap.INTAKE_ROLLER_VICTOR);
    intakeCoord = new Point2D.Double();

    rotatorTalon.setInverted(true);
    rotatorTalon.setSensorPhase(false); // Positive is inwards movement, negative is outward

    enablePercentOutput();
    setMotionMagicPIDValues();
  }

  // May need to set tolerance
  private void setMotionMagicPIDValues() {
    rotatorTalon.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
    rotatorTalon.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
    rotatorTalon.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
    rotatorTalon.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    rotatorTalon.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    rotatorTalon.configMotionCruiseVelocity(INTAKE_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    rotatorTalon.configMotionAcceleration(INTAKE_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_SECOND, kTimeoutMs);

    rotatorTalon.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputForward(MAX_BASE_OUTPUT, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputReverse(-MAX_BASE_OUTPUT, RobotConstants.kTimeoutMs);

    rotatorTalon.setNeutralMode(NeutralMode.Brake);
    rollerVictor.setNeutralMode(NeutralMode.Coast);
  }

  /**
   * Should set the setpoint for the Motion Magic on the intake
   */
  public void setMotionMagicSetpointAngle(double intakeAngle) {
    setpoint = convertIntakeDegreesToTicks(intakeAngle);
    SmartDashboard.putNumber("MotionMagicSetpoint", setpoint);
    rotatorTalon.set(ControlMode.MotionMagic, setpoint);
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
    rotatorTalon.setNeutralMode(mode);
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
    rollerVictor.set(ControlMode.PercentOutput, power);
  }
  /**
   * Converts an intake angle into ticks
   */
  private static int convertIntakeDegreesToTicks(double degrees) {
    //If straight up is 0 and going forward is positive
    // percentage * half rotation
    // TODO - Fix this. This is not going to make 0 at the top if the absolute encoder is not zero there.
    // Offset the quadrature readings accordingly in the constructor?
    return (int)((degrees / 360) * TICKS_PER_ROTATION);
  }

  /**
   * Converts intake ticks into an angle
   */
  private static double convertIntakeTicksToDegrees(double ticks) {
    //If straight up is 0 and going forward is positive
    // TODO - Fix this. This is not going to make 0 at the top if the absolute encoder is not zero there.
    // Offset the quadrature readings accordingly in the constructor?
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
