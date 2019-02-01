/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.drive.DriveMotionProfile;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import jaci.pathfinder.Waypoint;


public class BuildAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public BuildAuton(String[] autonSequence, Arm arm, boolean isReversed) {
    String start = autonSequence[0];
    String target1 = autonSequence[1];
    String height1 = autonSequence[2];
    String target2 = autonSequence[3];
    String target3 = autonSequence[4];
    String height3 = autonSequence[5];
    Waypoint[] points = new Waypoint[2];
    ArmState destination; 

    points[0] = RobotConstants.waypoints.get(start);
    points[1] = RobotConstants.waypoints.get(target1);
    destination = Arm.getArmState(LegalState.valueOf(height1)); // TODO change this
    addSequential(new DriveMotionProfile(points, isReversed));
    addParallel(new MoveArm(destination, arm));
    // addSequential: do appropriate thing with claw

    points[0] = RobotConstants.waypoints.get(target1);
    points[1] = RobotConstants.waypoints.get(target2);
    destination = Arm.getArmState(LegalState.GRAB_HATCH_LOADINGSTATION_FORWARD); // TODO change this
    addSequential(new DriveMotionProfile(points, isReversed));
    addParallel(new MoveArm(destination, arm));
    // addSequential: do appropriate thing with claw

    points[0] = RobotConstants.waypoints.get(target2);
    points[1] = RobotConstants.waypoints.get(target3);
    destination = Arm.getArmState(LegalState.valueOf(height3)); // TODO change this
    addSequential(new DriveMotionProfile(points, isReversed));
    addParallel(new MoveArm(destination, arm));
    // addSequential: do appropriate thing with claw
  }
}
