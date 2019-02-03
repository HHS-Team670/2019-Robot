/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;

import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.dataCollection.SensorCollection_PIDSource;

/**
 * Represents the climbing mechanism on the robot.
 * @author shaylandias
 */
public class Climber extends Subsystem {

  public static final int PISTON_ENCODER_FLAT = 0; // TODO set these
  public static final int PISTON_ENCODER_LEVEL_TWO = 0;
  public static final int PISTON_ENCODER_LEVEL_THREE = 0;

  // Motor that drives the two pistons in the back of the robot
  private WPI_TalonSRX backTalon;
  // Motor that drives the two pistons in the front of the robot. May be split into two controllers.
  private WPI_TalonSRX frontTalon;

  private PIDController frontPIDController;
  private PIDController backPIDController;

  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;

  private boolean frontPistonsRetracted, backPistonsRetracted;
  private boolean frontPistonRetractionInProgress, backPistonRetractionInProgress;

  private int climberEncoderTolerance = 10; //TODO Set this
  
  public static final double MINIMUM_PISTON_POWER = -0.2; //TODO: set this
  public static final double MAXIMUM_PISTON_POWER = 0.75; 

  public static final double WINCH_DIAMETER = 2.0;
  public static final double GEAR_RATIO = 47;
  public static final double CLIMBER_TICKS_PER_ROTATION = 4096;

  public static final double DISTANCE_BETWEEN_FRONT_AND_BACK_PISTONS_IN_INCHES = 11.834; 

  private MustangSensors sensors;

  public Climber(MustangSensors sensors) {    
    backTalon = new WPI_TalonSRX(RobotMap.BACK_CLIMBER_PISTON_CONTROLLER);
    frontTalon = new WPI_TalonSRX(RobotMap.FRONT_CLIMBER_PISTON_CONTROLLER);

    frontPistonsRetracted = true;
    backPistonsRetracted = true;

    frontPistonRetractionInProgress = false;
    backPistonRetractionInProgress = false;

    this.sensors = sensors;

    // TODO figure out if these motors need to be inverted.

    frontPIDController = new PIDController(P, I, D, F, new SensorCollection_PIDSource(frontTalon.getSensorCollection()), frontTalon); 
    backPIDController = new PIDController(P, I, D, F, new SensorCollection_PIDSource(frontTalon.getSensorCollection()), backTalon); 
  }

  /**
   * Drives the pistons down with percent power
   * 
   * @param frontPower the desired percent power for the front motors
   * @param backPower the desired percent power for the front motors
   */
  public void drivePistons(double frontPower, double backPower) {
    frontTalon.set(ControlMode.PercentOutput, frontPower);
    backTalon.set(ControlMode.PercentOutput, backPower);
  }

  /**
   * Sets the output range of the front pistons
   * 
   * @param minValue the minimum output for the front PID controller
   * @param maxValue the maximum output for the front PID controller
   */
  public void setFrontPistonOutputRange(double minValue, double maxValue){
    frontPIDController.setOutputRange(minValue, maxValue);
  }

   /**
   * Sets the output range of the back pistons
   * 
   * @param minValue the minimum output for the back PID controller
   * @param maxValue the maximum output for the back PID controller
   */
  public void setBackPistonOutputRange(double minValue, double maxValue){
    backPIDController.setOutputRange(minValue, maxValue);
  }

  /**
   * Sets a setpoint for the front PID controller
   * 
   * @param setpoint the desired setpoint for the front PID controller
   */
  public void setFrontPIDControllerSetpoint(int setpoint){
    setFrontPistonOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER);
    frontPIDController.setSetpoint(setpoint);
  }

  /**
   * Sets a setpoint for the back PID controller
   * 
   * @param setpoint the desired setpoint for the back PID controller
   */
  public void setBackPIDControllerSetpoint(int setpoint){
    setBackPistonOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER);
    backPIDController.setSetpoint(setpoint);
  }

  /**
   * Returns the position of the front pistons in ticks
   * 
   * @return The position of the front pistons in ticks
   */
  public int getFrontTalonPositionInTicks(){
    return frontTalon.getSensorCollection().getQuadraturePosition();
  }

  /**
   * Returns the position of the back pistons in ticks
   * 
   * @return The position of the back pistons in ticks
   */
  public int getBackTalonPositionInTicks(){
    return backTalon.getSensorCollection().getQuadraturePosition();
  }

  /**
   * Returns the position of the front pistons in inches
   * 
   * @return The position of the front pistons in inches
   */
  public double getFrontTalonPositionInInches() {
    return convertPistonTicksToInches(getFrontTalonPositionInTicks());
  }

   /**
   * Returns the position of the back pistons in inches
   * 
   * @return The position of the back pistons in inches
   */
  public double getBackTalonPositionInInches() {
    return convertPistonTicksToInches(getBackTalonPositionInTicks());
  }


  /**
   * Returns whether or not the command for retracting the front pistons has been called and finished
   * 
   * @return Whether or not the command for retracting the front pistons has been called and finished
   */
  public boolean getFrontPistonsRetracted(){
    return frontPistonsRetracted;
  }

  /**
   * Returns whether or not the command for retracting the back pistons has been called and finished
   * 
   * @return Whether or not the command for retracting the back pistons has been called and finished
   */
  public boolean getBackPistonsRetracted(){
    return backPistonsRetracted;
  }

  /**
   * Sets whether or not the command for retracting the front pistons has been called and finished
   * 
   * @param setRetracted Whether or not the command for retracting the front pistons has been called and finished
   */
  public void setFrontPistonsRetracted(boolean setRetracted){
    frontPistonsRetracted = setRetracted;
  }

  /**
   * Sets whether or not the command for retracting the back pistons has been called and finished
   * 
   * @param setRetracted Whether or not the command for retracting the back pistons has been called and finished
   */
  public void setBackPistonsRetracted(boolean setRetracted){
    backPistonsRetracted = setRetracted;
  }

    /**
   * Returns whether or not the command for retracting the front pistons has been called and is in progress
   * 
   * @return Whether or not the command for retracting the front pistons has been called and is in progress
   */
  public boolean getFrontPistonsRetractionInProgress(){
    return frontPistonRetractionInProgress;
  }

  /**
   * Returns whether or not the command for retracting the back pistons has been called and is in progress
   * 
   * @return Whether or not the command for retracting the back pistons has been called and is in progress
   */
  public boolean getBackPistonsRetractionInProgress(){
    return backPistonRetractionInProgress;
  }

  /**
   * Sets whether or not the command for retracting the front pistons has been called and is in progress
   * 
   * @param setRetractionInProgress Whether or not the command for retracting the front pistons has been called and is in progress
   */
  public void setFrontPistonsRetractionInProgress(boolean setRetractionInProgress){
    frontPistonRetractionInProgress = setRetractionInProgress;
  }

  /**
   * Sets whether or not the command for retracting the back pistons has been called and is in progress
   * 
   * @param setRetractionInProgress Whether or not the command for retracting the back pistons has been called and is in progress
   */
  public void setBackPistonsRetractionInProgress(boolean setRetractionInProgress){
    backPistonRetractionInProgress = setRetractionInProgress;
  }


  /**
   * Returns true if the front controller is on target and false if not
   */
  public boolean getFrontControllerOnTarget(){
    return frontPIDController.onTarget();
  }

  /**
   * Returns true if the back controller is on target and false if not
   */
  public boolean getBackControllerOnTarget(){
    return backPIDController.onTarget();
  }

  /**
   * Returns pitch calculated from the difference in the front and back piston encoders.
   * Returns negative if front is down and positive if front is up
   */
  public double getPitchFromEncoders(){
    double heightDifferenceFrontBackPistons = getFrontTalonPositionInInches() - getBackTalonPositionInInches();
    double pitch = Math.toDegrees(Math.asin(heightDifferenceFrontBackPistons / DISTANCE_BETWEEN_FRONT_AND_BACK_PISTONS_IN_INCHES));
    return pitch;
  }

  public static double convertPistonTicksToInches(int ticks){
    // Got this from Ben
    return ((WINCH_DIAMETER * Math.PI) / GEAR_RATIO) * CLIMBER_TICKS_PER_ROTATION;
  }

  /**
   * Enables the PIDControllers to get them running
   * 
   * @param setPoint the desired end goal of the piston climb
   */
  public void enableClimberPIDControllers(int setPoint){
    frontPIDController.setOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER);
    frontPIDController.setAbsoluteTolerance(climberEncoderTolerance);
    frontPIDController.setContinuous(false);
    frontPIDController.setSetpoint(setPoint);
    frontPIDController.enable();

    backPIDController.setOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER);
    backPIDController.setAbsoluteTolerance(climberEncoderTolerance);
    backPIDController.setContinuous(false);
    backPIDController.setSetpoint(setPoint);
    backPIDController.enable();
  }


  /**
   * Method to set output ranges of piston controllers in response to an input of tilt adjustment.
   * Will do nothing if the NavX is not connected!
   *
   * @param goingUp True if robot is climbing up and false if coming down
   * @param tiltTolerance The tolerance past which the robot is considered too unbalanced in either direction
   * @param tiltAdjustment The amount of adjustment desired for the robot
   */
  public void handleTilt(boolean goingUp, double tiltTolerance, double tiltAdjustment){
      if (goingUp) {
        // If tipped down (front is down)
        if (sensors.getPitchDouble() < -tiltTolerance) {
          setFrontPistonOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER + tiltAdjustment);
          setBackPistonOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER - tiltAdjustment);

          // If tipped up (front is up)
        } else if (sensors.getPitchDouble() > tiltTolerance) {
          setFrontPistonOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER - tiltAdjustment);
          setBackPistonOutputRange(MINIMUM_PISTON_POWER, MAXIMUM_PISTON_POWER + tiltAdjustment);
        }
      }
      // For going down
      else {
        // If tipped down (front is down)
        if (sensors.getPitchDouble() < -tiltTolerance) {
          setFrontPistonOutputRange(MINIMUM_PISTON_POWER + tiltAdjustment, MINIMUM_PISTON_POWER);
          setBackPistonOutputRange(MINIMUM_PISTON_POWER - tiltAdjustment, MINIMUM_PISTON_POWER);
          // If tipped up (front is up)
        } else if (sensors.getPitchDouble() > tiltTolerance) {
          setFrontPistonOutputRange(MINIMUM_PISTON_POWER - tiltAdjustment, MINIMUM_PISTON_POWER);
          setBackPistonOutputRange(MINIMUM_PISTON_POWER + tiltAdjustment, MINIMUM_PISTON_POWER);
        }
      }
  }

  @Override
  public void initDefaultCommand() {
    
  }


}
// TODO
// the scale is from the outer PID defined by the NAVX
// the innerPIDs only exist because we dont want to overshoot the encoder values
// scale factor is for the output range of the inner PIDs (pistons)
// retract - setpoint is 0, full power backward
// utility command to retract both at a low rate FIRST CHECK: command only if BOTH OF THEM are up (NOT just one)
// this command is just like the climb but down.