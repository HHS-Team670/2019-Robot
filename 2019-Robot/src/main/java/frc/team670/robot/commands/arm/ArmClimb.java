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
    //Move Arm to high position
    //Bring arm down
    addParallel(new SetElbowCurrentLimit(RobotConstants.TRIGGER_AMPS));
    
  }
}
