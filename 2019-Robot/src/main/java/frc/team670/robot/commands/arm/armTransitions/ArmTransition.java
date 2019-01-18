/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.utils.sort.Edge;

  /**
   * The base for arm transitions. All iterations of this class should be made so that they can be rerun each time initialize is called.
   * Essentially, initialize should reset all necessary values within the object.
   */
public abstract class ArmTransition extends CommandGroup implements Edge {

  /**
   * The LegalState that this Command will end at.
   */
  public abstract LegalState getDestination();

  /**
   * The LegalState that this Command should begin at.
   */
  public abstract LegalState getStart();

  /**
   * Returns the sum of the number of inches that the arm must move to cover the path to use as a method of weighting the transition.
   * Ticks that are traveled parallel to each other should not be counted. So if the arm is moving two joints at once, it should only
   * count the longer of the movements. Eventually, we can change this method to time spent once we have to robot built and can run
   * some empirical tests of the mechanisms.
   */
  public abstract int getLength();

}
