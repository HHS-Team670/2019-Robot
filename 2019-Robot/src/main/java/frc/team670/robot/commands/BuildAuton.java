/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.arm.movement.PlaceOrGrab;
import frc.team670.robot.commands.drive.DriveMotionProfile;
import frc.team670.robot.commands.drive.vision.AdvancedVisionPIDDrive;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;


public class BuildAuton extends CommandGroup {
  /**
   * Builds an auton sequence from a String[] sent by radio buttons on the dashboard
   * @param autonSequence a String[] of selected radio buttons
   * @param arm the arm object
   */
  public BuildAuton(String[] autonSequence, Arm arm) {
    String startDirection = autonSequence[0];
    String start = autonSequence[1];
    String target1 = autonSequence[2];
    String height1 = autonSequence[3];
    String target2 = autonSequence[4];
    String height2 = autonSequence[5];
    String target3 = autonSequence[6];
    String height3 = autonSequence[7];
    LegalState destination; 
    String fileName = "";

    // determines whether the robot starts facing backwards based on the first radio button
    boolean isRobotFacingBack = startDirection.equals("Backward");

    /* if the robot starts facing back, the first path and arm command will be going backwards
     * if the robot starts facing back, the second path and arm command will be going forwards
     * if the robot starts facing back, the third path and arm command will be going backwards */
    
    // gets the destination the arm should go to
    destination = getLegalState(target1, height1, true); 
    // finds the file corresponding to the path the robot should take
    fileName = start + "_" + target1 + ".pf1.csv";
    // drives along the path described by the file
    addSequential(new DriveMotionProfile(fileName, true));
    // moves the arm to the destination while driving
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    // drives to target using vision
    addSequential(new AdvancedVisionPIDDrive());
    // makes sure the arm is not above the robot
    if (destination != LegalState.NEUTRAL) {
        // places the element the robot is carrying
        addSequential(new PlaceOrGrab(true));
    }
 
    // repeat for second target with robot direction flipped
    destination = getLegalState(target2, height2, !isRobotFacingBack); 
    fileName = target1 + "_" + target2 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, !isRobotFacingBack));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new AdvancedVisionPIDDrive());
    if (destination != LegalState.NEUTRAL) addSequential(new PlaceOrGrab(false));

    // repeat for third target with original robot direction
    destination = getLegalState(target3, height3, isRobotFacingBack); 
    fileName = target2 + "_" + target3 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, isRobotFacingBack));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new AdvancedVisionPIDDrive());
    if (destination != LegalState.NEUTRAL) addSequential(new PlaceOrGrab(true));
  }

  /**
   * Returns the proper LegalState the arm should go to based on the target, height, and robot direction
   */
  private LegalState getLegalState(String target, String height, boolean isFacingBack) { 
    if (isFacingBack) {
        if (target.contains("Rocket") && (target.contains("1") || target.contains("3"))) {
            if (height.equals("MIDDLE")) {
                return LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK;
            } else if (height.equals("LOW")) {
                return LegalState.LOW_HATCH_BACK;
            }
        }
        else if (target.contains("Rocket") && target.contains("2")) {
            if (height.equals("MIDDLE")) {
                return LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK;
            } else if (height.equals("LOW")) {
                return LegalState.PLACE_BALL_ROCKET_LOW_BACK;
            }
        }
        else if (target.contains("Cargo")) {
            // if placing ball
            if (height.equals("MIDDLE")) {
                return LegalState.PLACE_BALL_CARGOSHIP_BACK;
            } 
            // if placing hatch
            else if (height.equals("LOW")) {
                return LegalState.LOW_HATCH_BACK;
            }
        }
    } else {
        if (target.contains("Rocket") && (target.contains("1") || target.contains("3"))) {
            if (height.equals("MIDDLE")) {
                return LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD;
            } else if (height.equals("LOW")) {
                return LegalState.LOW_HATCH_FORWARD;
            }
        }
        else if (target.contains("Rocket") && target.contains("2")) {
            if (height.equals("MIDDLE")) {
                return LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD;
            } else if (height.equals("LOW")) {
                return LegalState.PLACE_BALL_ROCKET_LOW_FORWARD;
            }
        }
        else if (target.contains("Cargo")) {
            // if placing ball
            if (height.equals("MIDDLE")) {
                return LegalState.PLACE_BALL_CARGOSHIP_FORWARD;
            } 
            // if placing hatch
            else if (height.equals("LOW")) {
                return LegalState.LOW_HATCH_FORWARD;
            }
        } else if (target.contains("Exchange")) {
            return LegalState.GRAB_BALL_LOADINGSTATION_FORWARD;
        }
    }
    return LegalState.NEUTRAL;
  }
}
