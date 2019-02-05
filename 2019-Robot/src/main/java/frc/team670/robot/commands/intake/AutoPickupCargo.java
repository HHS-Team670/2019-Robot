/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.claw.PickupBall;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Intake;

public class AutoPickupCargo extends CommandGroup {
  /**
   * @param arm The arm used in the command
   * @param intake The intake used in the command
   * @param claw The claw used in the command 
   * @param sensors The sensors used in the command
   */
  public AutoPickupCargo(Arm arm, Intake intake, Claw claw, MustangSensors sensors) {
    // addSequential(new MoveArm(Arm.getArmState(LegalState.INTAKE_BALL_INTAKE_FORWARD), arm)); // TODO change this to MoveArmToIntake so that it does it safely without hitting intake
    //runIntake will go until the IR in the claw gets tripped or 0.5 seconds after the Intake sensor has been tripped
    addSequential(new RunIntake(intake, sensors, true));
    addSequential(new PickupBall(claw));
    addParallel(new TimedRunIntake(5000, intake, true));
    addSequential(new WaitCommand(1));
    addSequential(new MoveArm(Arm.getArmState(LegalState.NEUTRAL), arm));
  }
}
