package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting at center and going under the switch to pick up 2 balls in a line
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author elisevbp
 */
public class Center2Line extends Path{

        public Center2Line(DriveBase driveBase) {
                super(
                        List.of(

                                //starts facing -90 deg
                                // new Pose2d(3.09, -2.628, Rotation2d.fromDegrees(0)),
                                // //TODO: same degree as Center3Line so if the angle for Center3Line is modifying change this too! 
                                // new Pose2d(4.932, -4.286, Rotation2d.fromDegrees(76.12)),
                                // new Pose2d(6.849, -4.063, Rotation2d.fromDegrees(111.3))

                                // new Pose2d(2.473, -2.4, Rotation2d.fromDegrees(0)),
                                // new Pose2d(5.872, -4.434, Rotation2d.fromDegrees(21.3)),
                                // new Pose2d(6.849, -4.063, Rotation2d.fromDegrees(21.3))

                                new Pose2d(3.09, -2.628, Rotation2d.fromDegrees(0)),
                                new Pose2d(4.92, -3.915, Rotation2d.fromDegrees(-45.5)),
                                new Pose2d(6.849, -4.063, Rotation2d.fromDegrees(21.3))

                        ), 
                driveBase);
        }
}
