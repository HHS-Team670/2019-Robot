/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.climb.PistonClimbWithTiltControl;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.Robot;

public class AbortClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AbortClimb() {
    if (Robot.climber.backPistonsDeployed && !Robot.climber.frontPistonsDeployed) {
      addSequential(new RetractFrontPistons());
      // addParallel: retract arm to neutral position
    } else {
      addSequential(new PistonClimbWithTiltControl(RobotConstants.PISTON_ENCODER_FLAT));
      // addParallel: retract arm to neutral position
    }
  }
}
