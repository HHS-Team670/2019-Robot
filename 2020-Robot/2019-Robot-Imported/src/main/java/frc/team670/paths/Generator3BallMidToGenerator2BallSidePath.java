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
 * Assuming the robot is already at the 3 power cells under the generator,
 * drives around the generator to the 2 power cells on the side of the
 * generator, near to the trench
 * 
 * @author meganchoy, ctychen
 */
public class Generator3BallMidToGenerator2BallSidePath extends Path {

    public Generator3BallMidToGenerator2BallSidePath(DriveBase driveBase) {
        super(
            List.of(
                new Pose2d(5.073, 5.266, Rotation2d.fromDegrees(89.081)),
                new Pose2d(5.8, 5.445, Rotation2d.fromDegrees(-65))
            ),
         driveBase);
    }
}