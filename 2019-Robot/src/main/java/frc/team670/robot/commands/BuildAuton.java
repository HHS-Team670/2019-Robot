/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import java.io.FileNotFoundException;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.drive.DriveMotionProfile;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
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
    ArmState destination; 
    String fileName = "";

    fileName = start + "_" + target1 + ".pf1.csv";
    destination = Arm.getArmState(LegalState.valueOf("READY_" + height1)); 
    try {
        addSequential(new DriveMotionProfile(fileName, isReversed));
        addParallel(new MoveArm(destination, arm));
    } catch (FileNotFoundException e) {
        e.printStackTrace();
        addSequential(new MoveArm(destination, arm));
    }
    addSequential(new PlaceElement(target1, height1));
    // addSequential: do appropriate thing with claw
    // addSequential: turn to proper angle to start next path?

    destination = Arm.getArmState(LegalState.valueOf("READY_" + height2)); 
    fileName = target1 + "_" + target2 + ".pf1.csv";
    try {
        addSequential(new DriveMotionProfile(fileName, isReversed));
        addParallel(new MoveArm(destination, arm));
    } catch (FileNotFoundException e) {
        e.printStackTrace();
        addSequential(new MoveArm(destination, arm));
    }    
    // addSequential: do appropriate thing with claw
    // addSequential: turn to proper angle to start next path?

    destination = Arm.getArmState(LegalState.valueOf("READY_" + height3)); 
    fileName = target2 + "_" + target3 + ".pf1.csv";
    try {
        addSequential(new DriveMotionProfile(fileName, isReversed));
        addParallel(new MoveArm(destination, arm));
    } catch (FileNotFoundException e) {
        e.printStackTrace();
        addSequential(new MoveArm(destination, arm));
    }
    // addSequential: do appropriate thing with claw
  }
}
