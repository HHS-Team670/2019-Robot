/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.armTransitions.HoldElbowDownWithCurrentLimit;
import frc.team670.robot.constants.RobotConstants;

public class ArmClimb extends CommandGroup {

  private static final int CURRENT_TO_HOLD = 15; // Test this out to figure out what to use

  public ArmClimb() {
    // addParallel(new MoveArm());
    //addSequential: Bring arm down to low position
    addParallel(new HoldElbowDownWithCurrentLimit(RobotConstants.TRIGGER_AMPS)); // These requirements won't work. 
    //addSequential: Telescope back                                     // We need to make an extension, elbow, and wrist Subsystem so we don't have conflicting requirements
    addParallel(new HoldElbowDownWithCurrentLimit(CURRENT_TO_HOLD));

    //addSequential: Move Arm to high position
    //addParallel: Telescope forward (all the way out)
    //addSequential: Bring arm down
    addParallel(new HoldElbowDownWithCurrentLimit(RobotConstants.TRIGGER_AMPS));
    //addSequential: Telescope back
    addParallel(new HoldElbowDownWithCurrentLimit(0));

    //addSequential: Move Arm to high position
  }

  @Override
  protected void end() {
    //Bring arm to high position
  }

  @Override
  protected void interrupted() {
    end();
  }
}
