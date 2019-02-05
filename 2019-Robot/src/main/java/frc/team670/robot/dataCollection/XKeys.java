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
import frc.team670.robot.commands.climb.controlClimb.CycleClimb;
import frc.team670.robot.commands.climb.pistonClimb.PistonClimbWithTiltControl;
import frc.team670.robot.commands.intake.AutoPickupCargo;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Climber;


/**
 * Listens on network tables to keys sent over by the XKeys keyboard and calls the corresponding commands
 * 
 * Link to XKeys bindings: https://docs.google.com/spreadsheets/d/1Y1cZvWabaVvush9LvfwKdRgCdmRTlztb67nWk5D-5x4/edit?usp=sharing
 * Link to Dashboard where XKeys are read in and values are sent over networktables: https://github.com/HHS-Team670/FRCDashboard 
 */
public class XKeys {

    private NetworkTableInstance instance;
    private NetworkTable table;
    private Command autonCommand;
    private ClimbHeight height;

    public XKeys() {
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");
        height = ClimbHeight.FLAT;

        table.addEntryListener("autonSequence", (table2, key2, entry, value, flags) -> {
            autonCommand = new BuildAuton(value.getStringArray(), Robot.arm);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-armstates", (table2, key2, entry, value, flags) -> {
            moveArm(Arm.getArmState(LegalState.valueOf(value.toString())));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-placing", (table2, key2, entry, value, flags) -> {
            if (value.toString().equals("place")) placeOrGrab(true);
            else if (value.toString().equals("grab")) placeOrGrab(false);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-intake", (table2, key2, entry, value, flags) -> {
            if (value.toString().equals("run_intake_in")) runIntake(true);
            else if (value.toString().equals("run_intake_out")) runIntake(false);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-autopickup", (table2, key2, entry, value, flags) -> {
            autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.toString().equals("set_climb_flat")) height = ClimbHeight.FLAT;
            else if (value.toString().equals("set_climb_2")) height = ClimbHeight.LEVEL2;
            else if (value.toString().equals("set_climb_3")) height = ClimbHeight.LEVEL3;
            
            if (value.toString().equals("cancel_arm_climb")) cancelArmClimb();
            if (value.toString().contains("cycle_climb")) nextStepArmClimb(height);
            if (value.toString().equals("piston_climb")) pistonClimb(height);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public Command getAutonCommand() {
        return autonCommand;
    }

    private void moveArm(ArmState state) {
        Scheduler.getInstance().add(new MoveArm(state, Robot.arm));
    }

    private void placeOrGrab(boolean isPlacing) {
        Scheduler.getInstance().add(new PlaceOrGrab(isPlacing));
    }

    private void runIntake(boolean runningIn) {
        Scheduler.getInstance().add(new RunIntake(Robot.intake, Robot.sensors, runningIn));
    }

    private void autoPickupBall() {
        Scheduler.getInstance().add(new AutoPickupCargo(Robot.arm, Robot.intake, Robot.claw, Robot.sensors));
    }

    private void nextStepArmClimb(ClimbHeight height) {
        if (height == ClimbHeight.FLAT) Scheduler.getInstance().add(new CycleClimb(Robot.arm, Robot.climber, Robot.sensors, Climber.PISTON_ENCODER_FLAT));
        else if (height == ClimbHeight.LEVEL2) Scheduler.getInstance().add(new CycleClimb(Robot.arm, Robot.climber, Robot.sensors, Climber.PISTON_ENCODER_LEVEL_TWO));
        else if (height == ClimbHeight.LEVEL3) Scheduler.getInstance().add(new CycleClimb(Robot.arm, Robot.climber, Robot.sensors, Climber.PISTON_ENCODER_LEVEL_THREE));
    }

    private void pistonClimb(ClimbHeight height) {
        int setpoint = 0;
        if (height == ClimbHeight.FLAT) setpoint = Climber.PISTON_ENCODER_FLAT;
        if (height == ClimbHeight.LEVEL2) setpoint = Climber.PISTON_ENCODER_LEVEL_TWO;
        if (height == ClimbHeight.LEVEL3) setpoint = Climber.PISTON_ENCODER_LEVEL_THREE;
        Scheduler.getInstance().add(new PistonClimbWithTiltControl(setpoint, Robot.climber, Robot.sensors));
    }

    private void cancelArmClimb() {
        Scheduler.getInstance().add(new CancelArmClimb(Robot.arm));
    }

    private enum ClimbHeight {
        FLAT, LEVEL2, LEVEL3;
    }
}