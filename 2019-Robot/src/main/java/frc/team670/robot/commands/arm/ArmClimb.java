/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.armTransitions.SetElbowCurrentLimit;
import frc.team670.robot.constants.RobotConstants;

public class ArmClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ArmClimb() {
    addParallel: new MoveArm()
    //addSequential: Bring arm down to low position
    addParallel(new SetElbowCurrentLimit(RobotConstants.TRIGGER_AMPS));
    //addSequential: Telescope back
    addParallel(new SetElbowCurrentLimit(0));


    //addSequential: Move Arm to high position
    //addParallel: Telescope forward (all the way out)
    //addSequential: Bring arm down
    addParallel(new SetElbowCurrentLimit(RobotConstants.TRIGGER_AMPS));
    //addSequential: Telescope back
    addParallel(new SetElbowCurrentLimit(0));

    //addSequential: Move Arm to high position
  }

  @Override
  protected void end(){
    //Bring arm to high position
  }
}
