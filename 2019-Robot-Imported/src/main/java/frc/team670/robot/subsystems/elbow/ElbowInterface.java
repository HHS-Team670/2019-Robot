package frc.team670.robot.subsystems.elbow;

/**
 * Allows for using an Elbow for unit testing
 */
public interface ElbowInterface {

    public void setMotionMagicSetpointAngle(double elbowSetpointAngle);

    public double getAngleInDegrees();

/**
   * Sets the output for the elbow motor 
   * @param output the desired output
   */
  public void setOutput(double output);

  /**
   * Returns the output current
   * 
   * @return the output current of the elbow motor
   */
  public double getOutputCurrent();

  /**
   * Sets the current limit for when the robot begins to climb
   */
  public void setClimbingCurrentLimit();

  /**
   * Resets the currnet limit to its normal value
   */
  public void setNormalCurrentLimit();
  
  /**
   * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position
   * @return true if the forward limit switch is closed, false if open
   */
  public boolean isForwardLimitPressed();

  /**
   * @return true if the forward limit switch is closed, false if open
   */
  public boolean isReverseLmitPressed();
  /**
   * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position)
   */
  public void setQuadratureEncoder(double encoderValue);

  /**
   * Sets the elbow to Current-based PID Control
   * @param current The current to control to. Positive means it will move forward, negative will move reverse.
   */
  public void setCurrentControl(int current);


}