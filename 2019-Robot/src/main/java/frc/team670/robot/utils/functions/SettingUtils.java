/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.functions;

import edu.wpi.first.wpilibj.PIDController;

/**
 * Contains utility functions for setting values on certain objects.
 */
public class SettingUtils {

  /**
   * Disables controller and releases its resources.
   */
  public static void releaseController(PIDController controller) {
    controller.disable();
    controller.free();
  }

}
