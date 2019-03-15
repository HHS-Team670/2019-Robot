/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.cameras.FlipDriverCameraMode;
import frc.team670.robot.commands.claw.CloseClaw;
import frc.team670.robot.commands.claw.OpenClaw;
import frc.team670.robot.commands.drive.teleop.FlipDriveAndCamera;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.utils.MustangController;
import frc.team670.robot.utils.MustangController.XboxButtons;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  // Controllers/Joysticks
  private MustangController driverController;

  private XKeys xkeys;

  // Buttons
  private JoystickButton toggleReverseDrive, toggleDriverCameraMode;
  private JoystickButton armToNeutral;  

  private JoystickButton dropBall, grabBall;

  public OI() {
    driverController = new MustangController(RobotMap.DRIVER_CONTROLLER_PORT);
    // operatorController = new MustangController(RobotMap.OPERATOR_CONTROLLER_PORT);
    xkeys = new XKeys();
    toggleReverseDrive = new JoystickButton(driverController, XboxButtons.LEFT_BUMPER);
    toggleReverseDrive.whenPressed(new FlipDriveAndCamera());
    toggleDriverCameraMode = new JoystickButton(driverController, XboxButtons.B);
    toggleDriverCameraMode.whenPressed(new FlipDriverCameraMode());
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
}
