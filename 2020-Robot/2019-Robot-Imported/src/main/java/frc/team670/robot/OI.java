/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import java.net.ProtocolException;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.CancelAllCommands;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.arm.movement.MoveExtensionBackUntilHitsLimitSwitch;
import frc.team670.robot.commands.arm.movement.PlaceOrGrab;
import frc.team670.robot.commands.cameras.FlipDriverCameraMode;
import frc.team670.robot.commands.claw.ToggleClaw;
import frc.team670.robot.commands.claw.YeetHeldItem;
import frc.team670.robot.commands.drive.teleop.FlipDriveAndCamera;
import frc.team670.robot.commands.drive.teleop.ToggleDriveSafe;
import frc.team670.robot.commands.intake.AutoPickupCargo;
import frc.team670.robot.commands.intake.ButtonRunIntake;
import frc.team670.robot.commands.intake.MoveIntakeToSetpointAngle;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.XboxButtons;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Controllers/Joysticks
  private MustangController driverController;
  private static Joystick operatorController  = new Joystick(RobotMap.OPERATOR_CONTROLLER_PORT);

  private XKeys xkeys;

  //operator buttons
  public static JoystickButton bringOutIntake = new JoystickButton(getOperatorController(), 2);
  public static JoystickButton bringInIntake = new JoystickButton(getOperatorController(), 4);
  public static JoystickButton runIntakeIn = new JoystickButton(getOperatorController(), 6);
  public static JoystickButton runIntakeOut = new JoystickButton(getOperatorController(), 5);
  // public static JoystickButton dropHeldItem = new JoystickButton(getOperatorController(), 11);
  public static JoystickButton toggleClaw = new JoystickButton(getOperatorController(), 1);
  public static JoystickButton autoPickupBall = new JoystickButton(getOperatorController(), 3);
  public static JoystickButton middleArm = new JoystickButton(getOperatorController(), 7);
  // Buttons
  public JoystickButton toggleReverseDrive, toggleDriverCameraMode, toggleChildSafe;
  public JoystickButton armToNeutral;  

  private JoystickButton dropBall, grabBall;

  private boolean intakeDeployed = false;

  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
    //xkeys = new XKeys();
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveAndCamera());
    toggleChildSafe = new JoystickButton(driverController, XboxButtons.X);
    toggleChildSafe.whenPressed(new ToggleDriveSafe());
    toggleDriverCameraMode = new JoystickButton(driverController, XboxButtons.B);
    toggleDriverCameraMode.whenPressed(new FlipDriverCameraMode());

    bringOutIntake.whenPressed(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED, Robot.intake));
    bringInIntake.whenPressed(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, Robot.intake));
    runIntakeIn.whenPressed(new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, true));
    runIntakeOut.whenPressed((new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, false)));
    // dropHeldItem.whenPressed(new ToggleClaw(Robot.claw));
    autoPickupBall.whenPressed(new AutoPickupCargo(Robot.arm, Robot.intake, Robot.claw, Robot.sensors));
    toggleClaw.whenPressed(new ToggleClaw(Robot.claw));
    middleArm.whenPressed(new MoveArm())

    
    armToNeutral = new JoystickButton(driverController, XboxButtons.A);
    armToNeutral.whenPressed(
      new InstantCommand() {
        protected void initialize() {
          Scheduler.getInstance().add(new MoveArm(Arm.getArmState(LegalState.NEUTRAL), Robot.arm));
        }
      }
    );

    // FOR TESTING OPENING/CLOSING the claw
    // dropBall = new JoystickButton(driverController, XboxButtons.Y);
    // dropBall.whenPressed(new OpenClaw(Robot.claw));
    // grabBall = new JoystickButton(driverController, XboxButtons.X);
    // grabBall.whenPressed(new CloseClaw(Robot.claw));

    // dropBall = new JoystickButton(driverController, XboxButtons.B);
    // dropBall.whenPressed(new InstantCommand() {
    //   protected void initialize() {
    //     Robot.claw.togglePush();
    //   }
    // });
  }
  /**
   * Sets the rumble on the driver controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleDriverController(double power, double time) {
    rumbleController(driverController, power, time);
  }

  /**
   * Sets the rumble on the operator controller
   * 
   * @param power The desired power of the rumble [0, 1]
   * @param time The time to rumble for in seconds
   */
  public void rumbleOperatorController(double power, double time) {
    // rumbleController(operatorController, power, time);
  }

  private void rumbleController(MustangController controller, double power, double time) {
    controller.rumble(power, time);
  }

  public MustangController getDriverController() {
    return driverController;
  }

  public static Joystick getOperatorController() {
    return operatorController;
  }

  // public MustangController getOperatorController() {
  //   return operatorController;
  // }

  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }

  public Command getSelectedAutonCommand() {
    SmartDashboard.putBoolean("auto-command", xkeys.getAutonCommand() == null);
    return xkeys.getAutonCommand();
  }

  //ARM COMMANDS 

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

private void runIntakeIn() {
  XKeys.toggleIn = !XKeys.toggleIn;

  if(XKeys.toggleIn){
    XKeys.toggleIn = true;
    XKeys.toggleOut = false;
  }

  if (XKeys.toggleIn) {
      System.out.println("Run Intake In command called");
      Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, true));
  } else {
      System.out.println("Run Intake In command canceled");
      Scheduler.getInstance().add(new ButtonRunIntake(Robot.intake, 0, true));
  }
}

private void runIntakeOut() {
    XKeys.toggleOut = !XKeys.toggleOut;

    if(XKeys.toggleOut){
      XKeys.toggleIn = false;
      XKeys.toggleOut = true;
    }
    
    if (XKeys.toggleOut) {
        new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, false);
    } else {
        new ButtonRunIntake(Robot.intake, 0, false);
    }
}

private void autoPickupBall() {
    Scheduler.getInstance().add(new AutoPickupCargo(Robot.arm, Robot.intake, Robot.claw, Robot.sensors));
}

private void cancelAllCommands() {
    Scheduler.getInstance().add(new CancelAllCommands(Robot.driveBase, Robot.arm, Robot.intake, Robot.claw));
}

private void dropHeldItem() {
    Scheduler.getInstance().add(new YeetHeldItem(Robot.claw, Robot.arm));
}

private void toggleClaw() {
    Scheduler.getInstance().add(new ToggleClaw(Robot.claw));
}

private void bringInIntake() {
    Scheduler.getInstance().add(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, Robot.intake));
}

private void bringOutIntake() {
    Scheduler.getInstance().add(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED, Robot.intake));
}
}
