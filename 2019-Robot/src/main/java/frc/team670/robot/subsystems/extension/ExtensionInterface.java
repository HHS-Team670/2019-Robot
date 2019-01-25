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
  public int getLengthTicks();
  
  /**
   * Gets the current Extension length in absolute inches with 0 at no extension.
   */
  public double getLengthInches();

  /**
   * Enables the PID Controller for extension
   */
  public void enableExtensionPIDController();

  /**
   * Modifies the setpoint for the PID Controller
   */
  public void setPIDControllerSetpoint(int setpoint);

  public void initDefaultCommand();

  public boolean isReverseLimitPressed();

  /**
   * @return true if forward limit switch closed, false if not
   */
  public boolean isForwardLimitPressed();
  
  /**
   * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position)
   */
  public void zero(double encoderValue);

  /**
   * Selects the PID Slot dedicated to MotionMagic to give it the correct PID Values
   */
  public void initializeMotionmagic();

  /**
   * Setup for movement and Motion Magic
   */
  public void setMotionMagicSetpoint(double extensionLength);

}