/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.extension.Extension;

/**
 * Add your docs here.
 */
public class TestExtension extends BaseExtension {

  double extensionLengthInInches;

  public TestExtension() {

  }

  @Override
  public double getLengthInches() {
    return extensionLengthInInches;
  }

  @Override
  public void setMotionMagicSetpointInInches(double extensionLengthInInches) {
    this.extensionLengthInInches = extensionLengthInInches;
  }

  @Override
  public void setCurrentLimit(int current) {
  }

  @Override
  public void enableCurrentLimit() {
  }

  @Override
  public int getLengthTicks() {
    return 0;
  }

  @Override
  public void disableCurrentLimit() {
  }

  @Override
  public void setOutput(double output) {
  }

  @Override
  public void enableExtensionPIDController() {
  }

  @Override
  public void setPIDControllerSetpointInInches(double setpointInInches) {
    extensionLengthInInches = setpointInInches;
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
  public void setQuadratureEncoder(double encoderValue) {
  }

  @Override
  public boolean getTimeout() {
    return false;
  }

  @Override
  public void stop() {

  }

  @Override
  public void moveByPercentOutput(double output) {

  }

  @Override
  public double getForwardSoftLimitInInches() {
    return 0;
  }

  @Override
  public double getReverseSoftLimitInInches() {
    return 0;
  }

  @Override
  public double getMotionMagicSetpoint() {
    return 0;
  }

  @Override
  public void enableBrakeMode() {

  }

  @Override
  public void enableCoastMode() {

  }

  @Override
  public boolean getForwardLimitSwitchTripped() {
    return false;
  }

  @Override
  public boolean getReverseLimitSwitchTripped() {
    return false;
  }

  @Override
  public double getArbitraryFeedForwardAngleMultiplier() {
    return 0;
  }

  @Override
  public void warnifLimitHit() {

  }

}
