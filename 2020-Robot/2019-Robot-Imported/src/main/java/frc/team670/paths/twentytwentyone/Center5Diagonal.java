package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting from the center of the port (in perspective of the driver) and going diagonally into the switch to pick up as many of 5 balls as possible
 * front of robot starts on initiation line
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author tarini
 */
public class Center5Diagonal extends Path{

        public Center5Diagonal (DriveBase driveBase) {
                super(
                        List.of(
                                // wider ending turn
                                new Pose2d(3.944, -2.628, Rotation2d.fromDegrees(0)),
                                // new Pose2d(5.662, -2.158, Rotation2d.fromDegrees(-5.03)),
                                // new Pose2d(6.849, -2.604, Rotation2d.fromDegrees(-39.46)), // experiment between wider or tighter turn
                                new Pose2d(6.268, -2.826, Rotation2d.fromDegrees(-53.88)), // experiment between wider or tighter turn
                                new Pose2d(6.614, -5.04, Rotation2d.fromDegrees(-129.34)) // experiment between wider or tighter turn

                                // new Pose2d(6.688, -5.028, Rotation2d.fromDegrees(-129.34)) // experiment with ending point (wider or tighter turn)

                                // tighter ending turn
                                /*
                                new Pose2d(3.944, -2.628, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.662, -2.158, Rotation2d.fromDegrees(-5.03)),
                                new Pose2d(6.367, -2.888, Rotation2d.fromDegrees(-65.28)), // experiment between wider or tighter turn
                                new Pose2d(6.033, -4.335, Rotation2d.fromDegrees(-159.174)) // experiment with ending point (wider or tighter turn)
                                */
                        ), 
                driveBase);
        }
}
