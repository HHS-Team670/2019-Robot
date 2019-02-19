/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.drive.pivot.NavXChangeableAnglePivot;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.MutableDouble;

public class VisionPurePursuitWithPivot extends CommandGroup {
  /**
   * Add your docs here.
   */
    
  public VisionPurePursuitWithPivot(DriveBase driveBase, MustangCoprocessor coprocessor, MustangSensors sensors,
  double spaceFromTarget, boolean isReversed, boolean lowTarget) {

    MutableDouble changeableAngle = new MutableDouble(0);

    addSequential(new VisionPurePursuit(driveBase, coprocessor, sensors, spaceFromTarget, isReversed, lowTarget, changeableAngle));
    addSequential(new NavXChangeableAnglePivot(changeableAngle, driveBase, sensors));
  }
}
