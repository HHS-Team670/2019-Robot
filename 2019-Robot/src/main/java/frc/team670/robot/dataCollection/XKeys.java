/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.BuildAuton;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.arm.movement.PlaceOrGrab;
import frc.team670.robot.commands.climb.armClimb.CancelArmClimb;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;


/**
 * Add your docs here.
 */
public class XKeys {

    private NetworkTableInstance instance;
    private NetworkTable table;
    private Command autonCommand;

    public XKeys() {
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");
        table.addEntryListener("autonSequence", (table2, key2, entry, value, flags) -> {
            autonCommand = new BuildAuton(value.getStringArray(), Robot.arm, false);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys", (table2, key2, entry, value, flags) -> {
            performAction(value.toString());
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public Command getAutonCommand() {
        return autonCommand;
    }

    private void performAction(String input) {
        if (input.toUpperCase().equals(input)) {
            Scheduler.getInstance().add(new MoveArm(Arm.getArmState(LegalState.valueOf(input)), Robot.arm));
        } else if (input.equals("place")) {
            Scheduler.getInstance().add(new PlaceOrGrab(true));
        } else if (input.equals("grab")) {
            Scheduler.getInstance().add(new PlaceOrGrab(false));
        } else if (input.equals("run_intake_in")) {
            Scheduler.getInstance().add(new RunIntake(Robot.intake, Robot.sensors, true));
        } else if (input.equals("run_intake_out")) {
            Scheduler.getInstance().add(new RunIntake(Robot.intake, Robot.sensors, false));
        } else if (input.equals("next_step_climb")) {
            // TODO: addSequential next step of the climbing process
        } else if (input.equals("cancel_arm_climb")) {
            Scheduler.getInstance().add(new CancelArmClimb(Robot.arm));
        }
    }
}
