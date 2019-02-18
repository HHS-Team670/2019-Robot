/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.arm.movement.ArmPathGenerator;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.claw.PickupBall;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Intake;

public class AutoPickupCargo extends CommandGroup {
  /**
   * @param arm     The arm used in the command
   * @param intake  The intake used in the command
   * @param claw    The claw used in the command
   * @param sensors The sensors used in the command
   */
  public AutoPickupCargo(Arm arm, Intake intake, Claw claw, MustangSensors sensors) {
    SmartDashboard.putString("current-command", "AutoPickupCargo");

    // runIntake will go until the IR in the claw gets tripped or 0.5 seconds after
    // the Intake sensor has been tripped
    CommandGroup moveArm = ArmPathGenerator.getPath(Arm.getStates().get(LegalState.GRAB_BALL_INTAKE), arm);
    addSequential(moveArm);
    addSequential(new RunIntakeInWithIR(intake, sensors));
    addSequential(new TimedRunIntake(Claw.TIME_TO_MOVE, intake, true));
    addSequential(new PickupBall(claw, arm));
    moveArm = ArmPathGenerator.getPath(Arm.getStates().get(LegalState.NEUTRAL), arm);
    addSequential(moveArm);
  }
}
