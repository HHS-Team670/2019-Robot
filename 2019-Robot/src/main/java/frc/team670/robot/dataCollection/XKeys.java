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
import frc.team670.robot.commands.arm.ToggleHeldItem;
import frc.team670.robot.commands.arm.movement.CancelArmMovement;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.arm.movement.MoveExtensionBackUntilHitsLimitSwitch;
import frc.team670.robot.commands.arm.movement.PlaceOrGrab;
import frc.team670.robot.commands.claw.ToggleClaw;
import frc.team670.robot.commands.claw.YeetHeldItem;
import frc.team670.robot.commands.drive.vision.CancelDriveBase;
import frc.team670.robot.commands.drive.vision.VisionPurePursuitWithPivot;
import frc.team670.robot.commands.intake.AutoPickupCargo;
import frc.team670.robot.commands.intake.ButtonRunIntake;
import frc.team670.robot.commands.intake.MoveIntakeToSetpointAngle;
import frc.team670.robot.commands.intake.RunIntakeInWithIR;
import frc.team670.robot.commands.intake.StopIntakeRollers;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.GrabBallLoadingStationBack;
import frc.team670.robot.subsystems.Arm.GrabBallLoadingStationForward;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Arm.PlaceBallCargoBack;
import frc.team670.robot.subsystems.Arm.PlaceBallCargoForward;
import frc.team670.robot.subsystems.Arm.PlaceGrabState;
import frc.team670.robot.subsystems.Arm.ReadyLowHatchBack;
import frc.team670.robot.subsystems.Arm.ReadyLowHatchForward;
import frc.team670.robot.subsystems.Arm.ReadyPlaceBallRocketLowBack;
import frc.team670.robot.subsystems.Arm.ReadyPlaceBallRocketLowForward;
import frc.team670.robot.subsystems.Arm.ReadyPlaceBallRocketMiddleBack;
import frc.team670.robot.subsystems.Arm.ReadyPlaceHatchRocketMiddleBack;
import frc.team670.robot.subsystems.Arm.ReadyPlaceHatchRocketMiddleForward;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.utils.Logger;


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
    private static boolean toggleIn = true, toggleOut = false;

    public XKeys() {
        SmartDashboard.putString("XKEYS", "XKeys constructor");
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");

        table.addEntryListener((table2, key2, entry, value, flags) -> {
            try {System.out.println("key pressed: " + value.getString());
            Logger.consoleLog("key pressed: " + value.getString());
            } catch (Exception e) {
                
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        table.addEntryListener("auton-sequence", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kStringArray) SmartDashboard.putString("auto-sequence", "not string array");
            SmartDashboard.putString("auto-sequence", "building auton");
            autonCommand = new BuildAuton(value.getStringArray(), Robot.arm);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-armstates", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            moveArm(getArmState(value.getString()));    
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-placing", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            if (s.equals("place")) placeOrGrab(true);
            else if (s.equals("grab")) placeOrGrab(false);
            else if (s.equals("drop_held_item")) dropHeldItem();
            else if (s.equals("toggle_held_item")) toggleHeldItem();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-intake", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            if (s.equals("run_intake_in_with_IR")) runIntakeInWithIR();
            else if (s.equals("toggle_intake_in")) runIntakeIn();
            else if (s.equals("toggle_intake_out")) runIntakeOut();
            else if (s.equals("bring_intake_in")) bringInIntake();
            else if (s.equals("bring_intake_out")) bringOutIntake();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-autopickup", (table2, key2, entry, value, flags) -> {
            autoPickupBall();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-visiondrive", (table2, key2, entry, value, flags) -> {
            visionDrive();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-claw", (table2, key2, entry, value, flags) -> {
            toggleClaw();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-extension", (table2, key2, entry, value, flags) -> {
            moveExtensionBackUntilHitsLimitSwitch();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        table.addEntryListener("xkeys-cancel", (table2, key2, entry, value, flags) -> {
            if (value.getType() != NetworkTableType.kString) return;
            String s = value.getString();
            if (s.equals("cancel_all")) cancelAllCommands();
            if (s.equals("cancel_arm")) cancelArmMovement();
            if (s.equals("cancel_drive")) cancelDriveBase();
            if (s.equals("cancel_intake")) cancelIntakeRollers();
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    public Command getAutonCommand() {
        return autonCommand;
    }

    private ArmState getArmState(String in) {
        LegalState legalState = null;
        if (in.equals("READY_PLACE_HATCH_ROCKET_MIDDLE_BACK")) legalState = LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK; 
        if (in.equals("READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD")) legalState = LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD;
        if (in.equals("READY_PLACE_BALL_ROCKET_MIDDLE_BACK")) legalState = LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK;
        if (in.equals("GRAB_BALL_LOADINGSTATION_BACK")) legalState = LegalState.GRAB_BALL_LOADINGSTATION_BACK;
        if (in.equals("READY_GRAB_HATCH_GROUND_BACK")) legalState = LegalState.GRAB_HATCH_GROUND_BACK;
        if (in.equals("GRAB_BALL_LOADINGSTATION_FORWARD")) legalState = LegalState.GRAB_BALL_LOADINGSTATION_FORWARD;
        if (in.equals("PLACE_BALL_CARGOSHIP_BACK")) legalState = LegalState.PLACE_BALL_CARGOSHIP_BACK;
        if (in.equals("PLACE_BALL_CARGOSHIP_FORWARD")) legalState = LegalState.PLACE_BALL_CARGOSHIP_FORWARD;
        if (in.equals("READY_LOW_HATCH_BACK")) legalState = LegalState.READY_LOW_HATCH_BACK;
        if (in.equals("READY_LOW_HATCH_FORWARD")) legalState = LegalState.READY_LOW_HATCH_FORWARD;
        if (in.equals("READY_PLACE_BALL_ROCKET_LOW_BACK")) legalState = LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK;
        if (in.equals("READY_PLACE_BALL_ROCKET_LOW_FORWARD")) legalState = LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD;
        if (in.equals("GRAB_BALL_GROUND_BACK")) legalState = LegalState.GRAB_BALL_GROUND_BACK;
        if (in.equals("GRAB_BALL_INTAKE")) legalState = LegalState.GRAB_BALL_INTAKE;
        if (in.equals("STOW")) legalState = LegalState.STOW;
        if (in.equals("NEUTRAL")) legalState = LegalState.NEUTRAL;

        return Arm.getArmState(legalState);
    }

    private void moveArm(ArmState state) {
        SmartDashboard.putString("ARMSTATE", state.toString());
        System.out.println(".................MoveArm called");
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

        if(toggleIn){
            toggleIn = true;
            toggleOut = false;
        }

        if (toggleIn) {
            System.out.println("Run Intake In command called");
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, true));
        } else {
            System.out.println("Run Intake In command canceled");
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, 0, true));
        }
    }

    private void runIntakeOut() {
        toggleOut = !toggleOut;

        if(toggleOut){
            toggleIn = false;
            toggleOut = true;
        }
        
        if (toggleOut) {
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, false));
        } else {
            Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, 0, false));
        }
    }

    private void autoPickupBall() {
        Scheduler.getInstance().add(new AutoPickupCargo(Robot.arm, Robot.intake, Robot.claw, Robot.sensors));
    }

    private void cancelAllCommands() {
        Scheduler.getInstance().add(new CancelAllCommands(Robot.driveBase, Robot.arm, Robot.intake, Robot.claw));
    }

    private void cancelIntakeRollers(){
        Scheduler.getInstance().add(new StopIntakeRollers(Robot.intake));
    }

    private void toggleHeldItem() {
        System.out.println("Toggle Held Item");
        Scheduler.getInstance().add(new ToggleHeldItem(Robot.arm));
    }

    private void cancelArmMovement(){
        Scheduler.getInstance().add(new CancelArmMovement(Robot.arm, Robot.intake, Robot.claw));
    }

    private void cancelDriveBase(){
        Scheduler.getInstance().add(new CancelDriveBase(Robot.driveBase));
    }

    private void visionDrive(){
        ArmState possiblePlaceGrabState = null;
        ArmState targetState = Arm.getTargetState();
        if(targetState instanceof GrabBallLoadingStationForward) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD);
        }
        else if (targetState instanceof GrabBallLoadingStationBack) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_BACK);
        }
        else if (targetState instanceof ReadyLowHatchForward) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.LOW_HATCH_FORWARD);
        }
        else if (targetState instanceof ReadyLowHatchBack) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.LOW_HATCH_BACK);
        }
        else if (targetState instanceof PlaceBallCargoForward) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.PLACE_BALL_CARGOSHIP_FORWARD);
        }
        else if (targetState instanceof PlaceBallCargoBack) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.PLACE_BALL_CARGOSHIP_BACK);
        }
        else if (targetState instanceof ReadyPlaceHatchRocketMiddleForward) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD);
        }
        else if (targetState instanceof ReadyPlaceHatchRocketMiddleBack) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK);
        }
        else if (targetState instanceof ReadyPlaceBallRocketLowForward) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.PLACE_BALL_ROCKET_LOW_FORWARD);
        }
        else if (targetState instanceof ReadyPlaceBallRocketLowBack) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.PLACE_BALL_ROCKET_LOW_BACK);
        }
        else if (targetState instanceof ReadyPlaceBallRocketMiddleBack) {
            possiblePlaceGrabState = Arm.getArmState(LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK);
        }
        else {
            Logger.consoleLog("Target state is not a possible to place state");
            return;
        }
        
        PlaceGrabState placeGrab = null;

        try {
            placeGrab = (PlaceGrabState) possiblePlaceGrabState;
        } catch (ClassCastException ex) {
            System.out.println("Not in a Place/Grab State. State: " + placeGrab.getClass().getName());
            return;
        }

        double distanceFromTarget = Math.abs(placeGrab.getDistanceFromTarget());
        boolean isReversed = !placeGrab.getIsFront();
        boolean isLow = placeGrab.getIsLowTarget();

        SmartDashboard.putString("vision-status", "");
        Scheduler.getInstance().add(new VisionPurePursuitWithPivot(Robot.driveBase, Robot.coprocessor, Robot.sensors, distanceFromTarget, isReversed, isLow));
    }

    private void dropHeldItem() {
        Scheduler.getInstance().add(new YeetHeldItem(Robot.claw, Robot.arm));
    }

    private void toggleClaw() {
        Scheduler.getInstance().add(new ToggleClaw(Robot.claw));
    }

    private void moveExtensionBackUntilHitsLimitSwitch(){
        Scheduler.getInstance().add(new MoveExtensionBackUntilHitsLimitSwitch(Robot.arm.getExtension()));
    }
    
    public static void setToggleIn(boolean toggleInBoolean){
        toggleIn = toggleInBoolean;
        toggleOut = !toggleInBoolean;
    }

    public static void setToggleOut(boolean toggleOutBoolean){
        toggleOut = toggleOutBoolean;
        toggleIn = !toggleOutBoolean;
    }

    public static void setBothToggles(boolean bothTogglesBoolean){
        toggleIn = bothTogglesBoolean;
        toggleOut = bothTogglesBoolean;
    }

    private void bringInIntake() {
        Scheduler.getInstance().add(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, Robot.intake));
    }

    private void bringOutIntake() {
        Scheduler.getInstance().add(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED, Robot.intake));
    }


}
