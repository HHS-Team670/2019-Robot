/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.cameras.FlipDriverCameraMode;
import frc.team670.robot.commands.claw.ToggleClaw;
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

  // Buttons
  private JoystickButton toggleReverseDrive, toggleDriverCameraMode, toggleChildSafe;
  private JoystickButton armToNeutral, armToRocketLowBack;  

  public static JoystickButton bringOutIntake = new JoystickButton(getOperatorController(), 2);
  public static JoystickButton bringInIntake = new JoystickButton(getOperatorController(), 4);
  public static JoystickButton runIntakeIn = new JoystickButton(getOperatorController(), 5);
  public static JoystickButton runIntakeOut = new JoystickButton(getOperatorController(), 6);
  // public static JoystickButton dropHeldItem = new JoystickButton(getOperatorController(), 11);
  public static JoystickButton toggleClaw = new JoystickButton(getOperatorController(), 1);
  public static JoystickButton autoPickupBall = new JoystickButton(getOperatorController(), 3);
  public static JoystickButton middleForwardArm = new JoystickButton(getOperatorController(), 7);
  public static JoystickButton backMiddleBall = new JoystickButton(getOperatorController(), 9);
  public static JoystickButton neutral = new JoystickButton(getOperatorController(), 11);
  public static JoystickButton stow = new JoystickButton(getOperatorController(), 12);

  private JoystickButton dropBall, grabBall;

  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys();
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveAndCamera());
    toggleChildSafe = new JoystickButton(driverController, XboxButtons.X);
    toggleChildSafe.whenPressed(new ToggleDriveSafe());
    toggleDriverCameraMode = new JoystickButton(driverController, XboxButtons.B);
    toggleDriverCameraMode.whenPressed(new FlipDriverCameraMode());
    armToNeutral = new JoystickButton(driverController, XboxButtons.A);
    toggleClaw.whenPressed(new ToggleClaw(Robot.claw));
    armToNeutral.whenPressed(
      new InstantCommand() {
        protected void initialize() {
          Scheduler.getInstance().add(new MoveArm(Arm.getArmState(LegalState.NEUTRAL), Robot.arm));
        }
      }
    );
    armToRocketLowBack = new JoystickButton(operatorController, 11);
    middleForwardArm.whenPressed(
      new InstantCommand() {
        protected void initialize() {
          Scheduler.getInstance().add(new MoveArm(getArmState("READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD"), Robot.arm));
        }
      }
    );

    backMiddleBall.whenPressed(
      new InstantCommand() {
        protected void initialize() {
          Scheduler.getInstance().add(new MoveArm(getArmState("READY_PLACE_BALL_ROCKET_MIDDLE_BACK"), Robot.arm));
        }
      }
    );

    neutral.whenPressed(
      new InstantCommand() {
        protected void initialize() {
          Scheduler.getInstance().add(new MoveArm(getArmState("NEUTRAL"), Robot.arm));
        }
      }
    );

    stow.whenPressed(
      new InstantCommand() {
        protected void initialize() {
          Scheduler.getInstance().add(new MoveArm(getArmState("STOW"), Robot.arm));
        }
      }
    );

    bringOutIntake.whenPressed(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED, Robot.intake));
    bringInIntake.whenPressed(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, Robot.intake));
    runIntakeIn.whenPressed(new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, true));
    runIntakeOut.whenPressed((new ButtonRunIntake(Robot.intake, Intake.RUNNING_POWER, false)));
    autoPickupBall.whenPressed(new AutoPickupCargo(Robot.arm, Robot.intake, Robot.claw, Robot.sensors));
  

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

  public boolean isQuickTurnPressed() {
    return driverController.getRightBumper();
  }

  public Command getSelectedAutonCommand() {
    SmartDashboard.putBoolean("auto-command", xkeys.getAutonCommand() == null);
    return xkeys.getAutonCommand();
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
}
