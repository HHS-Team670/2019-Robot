package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting at center and going under the switch to pick up 3 balls in a line
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author elisevbp
 */
public class Center3Line extends Path{

        public Center3Line(DriveBase driveBase) {
                super(
                        List.of(
                                new Pose2d(3.09, -2.628, Rotation2d.fromDegrees(0)),
                                new Pose2d(4.635, -3.717, Rotation2d.fromDegrees(-23.28)),
                                new Pose2d(6.577, -3.333, Rotation2d.fromDegrees(21.3))

                                //new Pose2d(5.736, -3.655, Rotation2d.fromDegrees(21.3)),
                                //TODO: tune 2nd waypoint to optimize turns 
                                // new Pose2d(4.4, -3.457, Rotation2d.fromDegrees(-62.3)),
                                // new Pose2d(6.577, -3.333, Rotation2d.fromDegrees(21.3))
                        ), 
                driveBase);
        }
}
