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
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Represents the Intake mechanism of the robot.
 */
public class Intake extends BaseIntake {

  public static final int INTAKE_ANGLE_IN = 0, INTAKE_ANGLE_DEPLOYED = 90;
  public static final double INTAKE_FIXED_LENGTH_IN_INCHES = 0, INTAKE_ROTATING_LENGTH_IN_INCHES = 0; //TODO set actual value
  private static final double MAX_BASE_OUTPUT = 0.75;
  private static final double kF = 0, kP = 0.1, kI = 0, kD = 0; //TODO figure out what these are
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0; //TODO Set this
  private static final int FORWARD_SOFT_LIMIT = 0, REVERSE_SOFT_LIMIT = 0; // TODO figure out the values in rotations
  private static final double RAMP_RATE = 0.1;

  private TalonSRX baseTalon;
  private VictorSPX rollerVictor;
  
  private Point2D.Double intakeCoord;
  public static double TICKS_PER_ROTATION = 4096; // Still needs to be set


  private static int INTAKE_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 15000; // TODO set this
  private static int INTAKE_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; // TODO set this

  public Intake() {
    baseTalon = new TalonSRX(RobotMap.INTAKE_BASE_TALON);
    rollerVictor = new VictorSPX(RobotMap.INTAKE_ROLLER_VICTOR);
    intakeCoord = new Point2D.Double();
    enableBaseMotionMagic();
  }

  // May need to set tolerance
  private void enableBaseMotionMagic() {
    baseTalon.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
    baseTalon.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
    baseTalon.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
    baseTalon.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    baseTalon.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    baseTalon.configMotionCruiseVelocity(INTAKE_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    baseTalon.configMotionAcceleration(INTAKE_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);

    baseTalon.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    baseTalon.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    baseTalon.configPeakOutputForward(MAX_BASE_OUTPUT, RobotConstants.kTimeoutMs);
    baseTalon.configPeakOutputReverse(-MAX_BASE_OUTPUT, RobotConstants.kTimeoutMs);

    // These thresholds stop the motor when limit is reached
    baseTalon.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    baseTalon.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);

    // Enable Safety Measures
    baseTalon.configForwardSoftLimitEnable(true);
    baseTalon.configReverseSoftLimitEnable(true);

    baseTalon.setNeutralMode(NeutralMode.Brake);
    rollerVictor.setNeutralMode(NeutralMode.Coast);

    baseTalon.configClosedloopRamp(RAMP_RATE);
    baseTalon.configOpenloopRamp(RAMP_RATE);
  }

  /**
   * Should set the setpoint for the Motion Magic on the intake
   */
  public void setMotionMagicSetpoint(double intakeTicks) {
    baseTalon.set(ControlMode.MotionMagic, intakeTicks);
  }

  /**
   * Should return the setpoint for the motion magic on the base motor
   */
  public double getMotionMagicSetpoint() {
    return baseTalon.getClosedLoopTarget();
  }

  /**
   * Should return the setpoint for the motion magic on the base motor
   */
  public Point2D.Double getMotionMagicDestinationCoordinates(){
    double x = INTAKE_ROTATING_LENGTH_IN_INCHES * Math.cos(convertIntakeTicksToDegrees(getMotionMagicSetpoint()));
    double y = INTAKE_FIXED_LENGTH_IN_INCHES + INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(convertIntakeTicksToDegrees(getMotionMagicSetpoint()));
    return new Point2D.Double(x, y);
  }

  public void setRotatorNeutralMode(NeutralMode mode) {
    baseTalon.setNeutralMode(mode);
  }

  /**
   * Returns the tick value of the base motor
   */
  public int getIntakePositionInTicks() {
    return baseTalon.getSensorCollection().getQuadraturePosition();
  }

  /**
   * Returns the intake angle in degrees
   */
  public double getIntakeAngleInDegrees() {
    return 0;
    // return MathUtils.convertIntakeTicksToDegrees(getIntakePositionInTicks());
  }

  public static int convertIntakeDegreesToTicks(int degrees) {
    return 0;
  }

  public static int convertIntakeTicksToDegrees(int ticks) {
    return 0;
  }

  /**
   * Returns the x, y coordinates of the top of the intake
   */
  public Point2D.Double getIntakeCoordinates(){
    double x = INTAKE_ROTATING_LENGTH_IN_INCHES * Math.cos(getIntakeAngleInDegrees());
    double y = INTAKE_FIXED_LENGTH_IN_INCHES + INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(getIntakeAngleInDegrees());
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
    if (runningIn) {
      SmartDashboard.putString("intake-status", "running-in");
    } else {
      power *= -1;
      SmartDashboard.putString("intake-status", "running-out");
    }
    rollerVictor.set(ControlMode.PercentOutput, power);
  }
  /**
   * Converts an intake angle into ticks
   */
  public static int convertIntakeDegreesToTicks(double degrees) {
    //If straight up is 0 and going forward is positive
    // percentage * half rotation

    // TODO - Fix this. This is not going to make 0 at the top if the absolute encoder is not zero there.
    // Offset the quadrature readings accordingly in the constructor?
    // Also why are you multiplying by 0.5, it is not half ticks per rotation
    return (int)((degrees / 180) * (0.5 * TICKS_PER_ROTATION));
  }

  /**
   * Converts intake ticks into an angle
   */
  public static double convertIntakeTicksToDegrees(double ticks) {
    //If straight up is 0 and going forward is positive
    // percentage * half degrees rotation

     // TODO - Fix this. This is not going to make 0 at the top if the absolute encoder is not zero there.
    // Offset the quadrature readings accordingly in the constructor?
    // Also why are you multiplying by 0.5, it is not half ticks per rotation
    return (ticks / (0.5 * TICKS_PER_ROTATION) * 180);
  }

  @Override
  public void initDefaultCommand() {

  }
}
