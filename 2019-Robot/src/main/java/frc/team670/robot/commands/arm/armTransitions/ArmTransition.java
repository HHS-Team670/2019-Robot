/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.subsystems.Arm.LegalState;

  /**
   * The base for arm transitions. All iterations of this class should be made so that they can be rerun each time initialize is called.
   * Essentially, initialize should reset all necessary values within the object.
   */
public abstract class ArmTransition extends CommandGroup {

  /**
   * The LegalState that this Command will end at.
   */
  public abstract LegalState getDestination();

  /**
   * The LegalState that this Command must begin at.
   */
  public abstract LegalState getStart();

}
