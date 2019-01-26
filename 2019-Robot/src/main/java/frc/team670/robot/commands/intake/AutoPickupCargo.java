/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.subsystems.Arm.LegalState;

public class AutoPickupCargo extends CommandGroup {
  /**
   * @param arm The arm used in the command
   * @param intake The intake used in the command
   * @param claw The claw used in the command 
   * @param sensors The sensors used in the command
   */
  public AutoPickupCargo(Arm arm, Intake intake, Claw claw, MustangSensors sensors) {
    addSequential(new MoveArm(Arm.getArmState(LegalState.INTAKE_BALL_INTAKE_FORWARD), arm));
    addParallel(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DOWN, intake));
    //runIntake will go until the IR in the claw gets tripped or 0.5 seconds after the Intake sensor has been tripped
    addSequential(new RunIntake(intake, sensors));
    addSequential(new PickupBall(claw));
    addSequential(new MoveArm(Arm.getArmState(LegalState.NEUTRAL), arm));
  }
}
