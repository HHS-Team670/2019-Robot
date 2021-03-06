package frc.team670.robot.subsystems.extension;

public interface ExtensionInterface {

     /**
   * Sets the peak current limit for the elbow motor.
   * @param current Current in amps
   */
  public void setCurrentLimit(int current);

  public void enableCurrentLimit();

  public void disableCurrentLimit();

  public void setOutput(double output);

  /**
   * Gets the current Extension length in absolute ticks with 0 at no extension.
   */
  public double getLengthInches();

  /**
   * Enables the PID Controller for extension
   */
  public void enableExtensionPIDController();

  /**
   * Modifies the setpoint for the PID Controller
   */
  public void setPIDControllerSetpointInInches(double setpointInInches);

  public void initDefaultCommand();

  public boolean isReverseLimitPressed();

  public int getLengthTicks();

  /**
   * @return true if forward limit switch closed, false if not
   */
  public boolean isForwardLimitPressed();
  
  /**
   * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position)
   */
  public void setQuadratureEncoder(double encoderValue);

  /**
   * Setup for movement and Motion Magic
   */
  public void setMotionMagicSetpointInInches(double extensionSetpointInInches);

}