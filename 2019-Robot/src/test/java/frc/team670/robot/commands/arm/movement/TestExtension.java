/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.extension.BaseExtension;;

/**
 * Add your docs here.
 */
public class TestExtension extends BaseExtension {

    double extensionLength;

    public TestExtension() {

    }

    @Override
    public double getLengthInches() {
        return extensionLength;
    }

    @Override
    public void setMotionMagicSetpoint(double extensionLength) {
        this.extensionLength = extensionLength;
    }
    
    @Override
    public void setCurrentLimit(int current) {
    }
  
    @Override
    public void enableCurrentLimit() {
    }
  
    @Override
    public void disableCurrentLimit() {
    }
  
    @Override
    public void setOutput(double output){
    }
  
    @Override
    public int getLengthTicks() {
      return 0;
    }
  
    @Override
    public void enableExtensionPIDController() {
    }
  
    @Override
    public void setPIDControllerSetpoint(int setpoint) {
    }
  
    @Override
    public boolean isReverseLimitPressed() {
      return false;
    }
  
    @Override
    public boolean isForwardLimitPressed() {
      return false;
    }
    
    @Override
    public void zero(double encoderValue) {
    }

}
