/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

/**
 * Add your docs here.
 */
public interface TunableSubsystem {

    public boolean getTimeout();

    public void enablePercentOutput();

    public void rotatePercentOutput(double output);

}
