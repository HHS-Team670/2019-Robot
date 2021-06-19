/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.MutableDouble;
import frc.team670.robot.commands.drive.vision.VisionPurePursuitV2;

public class VisionDrive extends CommandGroup {
  /**
   * Add your docs here.
   */
    
  public VisionDrive(DriveBase driveBase, MustangCoprocessor coprocessor, MustangSensors sensors,
  double spaceFromTarget, boolean isReversed, boolean lowTarget, MutableDouble changeableAngle) {

    double[] visionData = new double[]{RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE};

    addSequential(new CollectVisionData(visionData, coprocessor, lowTarget, isReversed, driveBase));
    addSequential(new VisionPurePursuitV2(driveBase, sensors, isReversed, changeableAngle, spaceFromTarget, coprocessor));   
  }
}