/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Pass this object into PIDControllers as the PIDSource if you do not want the controller directly controlling motors.
 */
public class NullPIDOutput implements PIDOutput {

   @Override
   public void pidWrite(double output) {

   }

}