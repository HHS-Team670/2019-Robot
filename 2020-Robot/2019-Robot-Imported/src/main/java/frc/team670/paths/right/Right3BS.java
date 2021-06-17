/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.paths.right;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * Trajectory starting on the line and to the right of your power port and facing the 3 Power Cells under
 * the middle of the generator.
 * 
 * @author meganchoy, ctychen, arnavyk
 */
public class Right3BS extends Path{

    public Right3BS(DriveBase driveBase){
        super(
            List.of(
                new Pose2d(3.8, -1.005,  Rotation2d.fromDegrees(0)),
                new Pose2d(4.283, -3.276,  Rotation2d.fromDegrees(-80.354)),
                new Pose2d(5.679, -4.081, Rotation2d.fromDegrees(22.255))
            ),
            driveBase);
    }
}
