package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting to the left of the port (in perspective of the driver) and going under the switch to pick up 3 balls in a line
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author megchoy
 */
public class Left3Line extends Path{

        public Left3Line(DriveBase driveBase) {
                super(
                        List.of(
                                new Pose2d(3.201, -3.717, Rotation2d.fromDegrees(0)),
                                new Pose2d(6.671, -3.242, Rotation2d.fromDegrees(21.3))
                        ), 
                driveBase);
        }
}
