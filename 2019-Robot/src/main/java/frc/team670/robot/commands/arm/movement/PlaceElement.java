/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;

/**
 * Add your docs here.
 */
public class PlaceElement extends InstantCommand {
  /**
   * Add your docs here.
   */
  public PlaceElement(String target, String height) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    ArmState current = Robot.arm.getCurrentState();
    ArmState out = null;
    if (current.toString().contains("ROCKET_MIDDLE_FORWARD")) {
      out = Robot.arm.getArmState(LegalState.valueOf("PLACE_HATCH_ROCKET_MIDDLE_FORWARD"));
    } else if (current.toString().contains("CARGOSHIP_BACK")) {

    } 
    new MoveArm(out, Robot.arm);
  }

}
