/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths.left;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

public class LeftThenForward extends Path{

    public LeftThenForward(DriveBase driveBase){
        super(
                List.of(
                    new Pose2d(3.953, -3.915, Rotation2d.fromDegrees(0)),
                    new Pose2d(7.053, -3.915, Rotation2d.fromDegrees(0)) //Forward 2 meters
                    ),
            driveBase);
    }
}
