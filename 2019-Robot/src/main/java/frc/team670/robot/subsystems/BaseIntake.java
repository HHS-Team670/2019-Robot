/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import java.awt.geom.Point2D;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public abstract class BaseIntake extends RotatingSubsystem {
  
  public BaseIntake(TalonSRX rotatorTalon, double arbitrary_feedforward_constant, int forward_soft_limit, int reverse_soft_limit, boolean timeout, int QUAD_ENCODER_MIN, int QUAD_ENCODER_MAX, int CONTINUOUS_CURRENT_LIMIT, int PEAK_CURRENT_LIMIT, int offsetFromEncoderZero){
    super(rotatorTalon, arbitrary_feedforward_constant, forward_soft_limit, reverse_soft_limit, timeout, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, offsetFromEncoderZero);
  }

  @Override
  //public abstract void initDefaultCommand();

  /**
   * Should return the setpoint coordinates for the motion magic on the base motor
   */
  public abstract Point2D.Double getMotionMagicDestinationCoordinates();

  public abstract void setRotatorNeutralMode(NeutralMode mode);

  public void runIntakeUsingCurrent(int current) {
    
  }
  
  /** 
   * Returns the x, y coordinates of the top of the intake
   */
  public abstract Point2D.Double getIntakeCoordinates();


  /**
   * Runs the intake at a given percent power
   * 
   * @param percentOutput The desired percent power for the rollers to run at [-1, 1]
   */
  public abstract void runIntake(double power, boolean runningIn);

  public boolean isDeployed() {
    return (getAngleInDegrees() > 0);
  }

  public void stopRollers() {
  }

}