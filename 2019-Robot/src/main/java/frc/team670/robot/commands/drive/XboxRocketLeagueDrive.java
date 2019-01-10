/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;
import frc.team670.robot.utils.functions.JoystickUtils;
// import frc.robot.utils.XboxButtons;
 

/**
 * Drives the Robot using Xbox controls like the game Rocket League. Triggers control speed, stick is for steering.
 * @author lakshbhambhani
 */
public class XboxRocketLeagueDrive extends InstantCommand {

   private final boolean smoothRocketLeagueSteer, smoothRocketLeagueTrigger;
   private static boolean reversed;
   private static String state;

  public XboxRocketLeagueDrive() {
    super();
    smoothRocketLeagueSteer = true;
    smoothRocketLeagueTrigger = true;
    reversed = false;
    requires(Robot.driveBase);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    // Sets the speed to the reading given by the trigger axes on the controller. Left is positive, but we multiply
    // by -1 to reverse that because we want right trigger to correspond to forward.
    double speed = -1 * (Robot.oi.getDriverController().getLeftTriggerAxis() - Robot.oi.getDriverController().getRightTriggerAxis()); 
    double steer = Robot.oi.getDriverController().getLeftStickX(); 


    // Decides whether or not to smooth the Steering and Trigger. Smoothing helps reduce jerkiness when driving.
    if(smoothRocketLeagueSteer){
      steer = JoystickUtils.smoothInput(steer);
    }
    if(smoothRocketLeagueTrigger){
      speed = JoystickUtils.smoothInput(speed);
    }

    if(Robot.oi.isReversed()) {
      if(state.equals("enabled")) {
        XboxRocketLeagueDrive.setDriveForward();
      }
      else if(state.equals("disabled")) {
        XboxRocketLeagueDrive.setDriveReversed();
      }
    }

    if(reversed) {
      steer *= -1;
      speed *= -1;
    }

    if(Robot.oi.isQuickTurnPressed()){
      Robot.driveBase.curvatureDrive(speed, steer, Robot.oi.isQuickTurnPressed());
    } else {
      if (speed < 0.00001){
        Robot.driveBase.curvatureDrive(speed, -1 * steer, Robot.oi.isQuickTurnPressed());
      } else {
        Robot.driveBase.curvatureDrive(speed, steer, Robot.oi.isQuickTurnPressed());
      }
    }
  }

  public static boolean isDriveReversed() {
    if(state.equals("enabled")) {
      reversed = true;
    }
    else if(state.equals("disabled"){
      reversed = false;
    }
    return reversed;
  }

  public static void setDriveReversed() {
    reversed = true;
    state = "enabled";
  }

  public static void setDriveForward() {
    reversed = false;
    state = "disabled";
  }

}