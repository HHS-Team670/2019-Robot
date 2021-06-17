/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths.center;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Trajectory starting on the line closest to your score port (robot facing towards your own driverstation)
 * and facing the 3 Power Cells under the middle of the generator.
 * 
 * @author meganchoy, ctychen
 */
public class Center3BS extends Path{

    public Center3BS(DriveBase driveBase) {
        super(
            List.of(
                new Pose2d(3.986, -2.4, Rotation2d.fromDegrees(0)),
                new Pose2d(4.083, -3.721, Rotation2d.fromDegrees(-40)),
                new Pose2d(5.299, -4.353, Rotation2d.fromDegrees(26))
            ),
        driveBase);
    }
}