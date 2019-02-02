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
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;


public class BuildAuton extends CommandGroup {
  /**
   * Add your docs here.
   */
  public BuildAuton(String[] autonSequence, Arm arm, boolean isReversed) {
    String start = autonSequence[0];
    String target1 = autonSequence[1];
    String height1 = autonSequence[2];
    String target2 = autonSequence[3];
    String height2 = autonSequence[4];
    String target3 = autonSequence[5];
    String height3 = autonSequence[6];
    LegalState destination; 
    String fileName = "";

    destination = getLegalState(target1, height1); 
    fileName = start + "_" + target1 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, isReversed));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new PlaceOrGrab(destination, true));
    // addSequential: turn to proper angle to start next path?
 
    destination = getLegalState(target2, height2); 
    fileName = target1 + "_" + target2 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, isReversed));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new PlaceOrGrab(destination, false));
    // addSequential: turn to proper angle to start next path?

    destination = getLegalState(target3, height3); 
    fileName = target2 + "_" + target3 + ".pf1.csv";
    addSequential(new DriveMotionProfile(fileName, isReversed));
    addParallel(new MoveArm(Arm.getArmState(destination), arm));
    addSequential(new PlaceOrGrab(destination, true));
  }

  // TODO account for direction (FRONT or BACK)
  private LegalState getLegalState(String target, String height) {
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
    return null;
  }
}
