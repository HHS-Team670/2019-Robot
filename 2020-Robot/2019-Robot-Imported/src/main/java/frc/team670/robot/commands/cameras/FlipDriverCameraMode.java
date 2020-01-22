/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.cameras;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;

/**
 * Flips between single camera and double camera on the dashboard
 */
public class FlipDriverCameraMode extends InstantCommand {
  /**
   * Add your docs here.
   */
  private static boolean doubleCamera = true;

  public FlipDriverCameraMode() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    doubleCamera = !doubleCamera;
    SmartDashboard.putString("driver-camera-mode", doubleCamera ? "double" : "single");
    Robot.oi.rumbleDriverController(0.7, 0.2);
  }

}
