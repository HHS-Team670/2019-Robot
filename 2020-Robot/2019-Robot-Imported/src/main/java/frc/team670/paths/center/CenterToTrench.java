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
 * Trajectory starting on intitiation line closest to your score port (facing towards your driverstation)
 * and going through the trench
 * 
 * @author meganchoy, ctychen
 */
public class CenterToTrench extends Path {

    public CenterToTrench(DriveBase driveBase) {
        super(
            List.of(
                new Pose2d(3.986, -2.4, Rotation2d.fromDegrees(0)),
                new Pose2d(4.156, -1.483, Rotation2d.fromDegrees(70.54)),
                new Pose2d(5.159, -0.745, Rotation2d.fromDegrees(0))
                ),
        driveBase);
    }
}
