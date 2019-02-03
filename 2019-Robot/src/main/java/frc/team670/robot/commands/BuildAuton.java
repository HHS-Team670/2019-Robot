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
   * Add your docs here.
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

    boolean isRobotFacingBack = startDirection.equals("Backward");

    destination = getLegalState(target1, height1, isRobotFacingBack); 
    fileName = start + "_" + target1 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, isRobotFacingBack));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new AdvancedVisionPIDDrive());
    if (destination != LegalState.NEUTRAL) addSequential(new PlaceOrGrab(true));
 
    destination = getLegalState(target2, height2, !isRobotFacingBack); 
    fileName = target1 + "_" + target2 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, !isRobotFacingBack));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new AdvancedVisionPIDDrive());
    if (destination != LegalState.NEUTRAL) addSequential(new PlaceOrGrab(false));

    destination = getLegalState(target3, height3, isRobotFacingBack); 
    fileName = target2 + "_" + target3 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, isRobotFacingBack));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new AdvancedVisionPIDDrive());
    if (destination != LegalState.NEUTRAL) addSequential(new PlaceOrGrab(true));
  }

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
        }
    }
    return LegalState.NEUTRAL;
  }
}
