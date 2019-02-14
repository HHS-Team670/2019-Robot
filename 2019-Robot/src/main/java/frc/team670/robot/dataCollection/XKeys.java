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
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.BuildAuton;
import frc.team670.robot.commands.CancelAllCommands;
import frc.team670.robot.commands.arm.movement.CancelArmMovement;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.arm.movement.PlaceOrGrab;
import frc.team670.robot.commands.climb.armClimb.CancelArmClimb;
import frc.team670.robot.commands.climb.controlClimb.CycleClimb;
import frc.team670.robot.commands.climb.pistonClimb.AbortRobotPistonClimb;
import frc.team670.robot.commands.climb.pistonClimb.PistonClimbWithTiltControl;
import frc.team670.robot.commands.drive.vision.CancelDriveBase;
import frc.team670.robot.commands.drive.vision.VisionPurePursuit;
import frc.team670.robot.commands.intake.AutoPickupCargo;
import frc.team670.robot.commands.intake.ButtonRunIntake;
import frc.team670.robot.commands.intake.RunIntakeInWithIR;
import frc.team670.robot.commands.intake.StopIntakeRollers;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Arm.PlaceGrabState;
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

    private boolean intakeRunning = true;
    private boolean toggleIn = true, toggleOut = false;

    public XKeys() {
        SmartDashboard.putString("XKEYS", "XKeys constructor");
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");
        height = ClimbHeight.FLAT;

        table.addEntryListener("autonSequence", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kStringArray) return;
            autonCommand = new BuildAuton(value.getStringArray(), Robot.arm);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-armstates", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            moveArm(Arm.getArmState(LegalState.valueOf(s)));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-placing", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            SmartDashboard.putString("XKEYS", "placing listener");
            if (s.equals("place")) placeOrGrab(true);
            else if (s.equals("grab")) placeOrGrab(false);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-intake", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            if (s.equals("run_intake_in_with_IR")) runIntakeInWithIR();
            else if (s.equals("toggle_intake_in")) runIntakeIn();
            else if (s.equals("toggle_intake_out")) runIntakeOut();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-autopickup", (table2, key2, entry, value, flags) -> {
            autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-climber", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            if (s.equals("set_climb_flat")) height = ClimbHeight.FLAT;
            else if (s.equals("set_climb_2")) height = ClimbHeight.LEVEL2;
            else if (s.equals("set_climb_3")) height = ClimbHeight.LEVEL3;
            
            if (s.contains("cycle_climb")) nextStepArmClimb(height);
            else if (s.equals("piston_climb")) pistonClimb(height);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-visionDrive", (table2, key2, entry, value, flags) -> {
            visionDrive();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            if (s.equals("cancel_all")) cancelAllCommands();
            if (s.equals("cancel_arm")) cancelArmMovement();
            if (s.equals("cancel_drive")) cancelDriveBase();
            if (s.equals("cancel_intake")) cancelIntakeRollers();
            if (s.equals("cancel_arm_climb")) cancelArmClimb();
            if (s.equals("cancel_piston_climb")) cancelPistonClimb();
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

    private void runIntakeInWithIR() {
        Scheduler.getInstance().add(new RunIntakeInWithIR(Robot.intake, Robot.sensors));
    }

    private void runIntakeIn() {
        toggleIn = !toggleIn;

        if(toggleOut){
            toggleIn = true;
            toggleOut = false;
        }

        if (toggleIn) {
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, RunIntakeInWithIR.RUNNING_POWER, true));
        } else {
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, 0, true));
        }
    }

    private void runIntakeOut() {
        toggleOut = !toggleOut;

        if(toggleIn){
            toggleIn = false;
            toggleOut = true;
        }
        
        if (toggleOut) {
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, RunIntakeInWithIR.RUNNING_POWER, false));
        } else {
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, 0, false));
        }
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

    private void cancelAllCommands() {
        Scheduler.getInstance().add(new CancelAllCommands(Robot.driveBase, Robot.arm, Robot.intake, Robot.claw, Robot.climber));
    }

    private void cancelIntakeRollers(){
        Scheduler.getInstance().add(new StopIntakeRollers(Robot.intake));
    }

    private void cancelPistonClimb(){
        Scheduler.getInstance().add(new AbortRobotPistonClimb(Robot.climber, Robot.arm, Robot.sensors));
    }

    private void cancelArmMovement(){
        Scheduler.getInstance().add(new CancelArmMovement(Robot.arm, Robot.intake, Robot.claw));
    }

    private void cancelDriveBase(){
        Scheduler.getInstance().add(new CancelDriveBase(Robot.driveBase));
    }

    private void visionDrive(){
        PlaceGrabState placeGrabState = null;
        try {
             placeGrabState = (PlaceGrabState) Arm.getCurrentState();
        } catch (ClassCastException ex) {
            return;
        }

        double distanceFromTarget = placeGrabState.getDistanceFromTarget();
        boolean isReversed = !placeGrabState.getIsFront();

        Scheduler.getInstance().add(new VisionPurePursuit(Robot.driveBase, Robot.coprocessor, Robot.sensors, distanceFromTarget, isReversed));
    }
    private enum ClimbHeight {
        FLAT, LEVEL2, LEVEL3;
    }

}
