/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.team670.robot.Robot;
import frc.team670.robot.commands.climb.pistonClimb.MoveFrontPistonsToSetpoint;
import frc.team670.robot.commands.climb.pistonClimb.PistonClimbWithTiltControl;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.constants.RobotConstants;

/**
 * Command to run if there is very little time left in the match, and the driver just wants to bring the robot down.
 * If the front pistons have been partially retracted, it will deploy them to match the back pistons, then it will
 * bring the robot down
 */
public class AbortRobotPistonClimb extends CommandGroup {

  public AbortRobotPistonClimb(Climber climber) {
    // addParallel(new MoveArm()); TODO: Move arm to stowed state so it doesn't hit anything on the way down

    /*
     * If the call to retract has been called and is halfway through, this should
     * set the front pistons down so they are on the same level as the back pistons.
     * If that call has been finished, this block will never run.
     */
    if (climber.getFrontPistonsRetractionInProgress()) {
      addSequential(new MoveFrontPistonsToSetpoint(climber.getBackTalonPositionInTicks(), climber));
      climber.setFrontPistonsRetractionInProgress(false);
    }

    /*
     * If the front pistons have not already been retracted, in which case the
     * driver should just drive onto the platform, this should bring both pistons
     * down to flat
     */
    if (!Robot.climber.getFrontPistonsRetracted()) {
      addSequential(new PistonClimbWithTiltControl(RobotConstants.PISTON_ENCODER_FLAT, Robot.climber));
    }
  }
}
