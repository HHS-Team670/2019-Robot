package frc.team670.robot.subsystems.wrist;

public interface WristInterface {

    public void enableCurrentLimit();
    
      public void disableCurrentLimit();
    
      public void setOutput(double output);
    
      public int getPositionTicks();
      
      public double getAngle();
    
      public void initDefaultCommand();
    
      /**
      * @return true if the forward limit switch is closed, false if open
      */
      public boolean isForwardLimitPressed();
      
      /**
       * @return true if the forward limit switch is closed, false if open
       */
      public boolean isReverseLimitPressed();
      
    
    
      /**
      * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position)
      */
      public void zero(double encoderValue);
    
      /**
       * Setup for movement and Motion Magic
       */
      public void setMotionMagicSetpoint(double wristAngle);
}