/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Assuming the robot is already at the 2 power cells at the side of the
 * generator, drive to through the trench, and end facing the initiation line
 * 
 * @author meganchoy, ctychen
 */
public class Generator2BallSideToTrenchPath extends Path {

    public Generator2BallSideToTrenchPath(DriveBase driveBase) {
        super(
            List.of(
                new Pose2d(7.138, 6.178, Rotation2d.fromDegrees(23.92)),
                new Pose2d(7.99, 6.816, Rotation2d.fromDegrees(68.689)),
                new Pose2d(5.555, 7.437, Rotation2d.fromDegrees(180))
            ), 
        driveBase);
    }
}