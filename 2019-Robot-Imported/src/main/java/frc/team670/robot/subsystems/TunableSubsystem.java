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

    /**
     * Enables percent output to shut off other movement of the Subsystem (note this means it will stop holding itself up).
     */
    public void stop();

    /**
     * Moves the Subsystem using percent outpu
     */
    public void moveByPercentOutput(double output);

}
