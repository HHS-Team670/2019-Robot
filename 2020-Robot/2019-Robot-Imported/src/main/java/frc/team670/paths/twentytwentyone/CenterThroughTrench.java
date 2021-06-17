package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * first part of center trench trajectories
 * Trajectory starting from the center of the field in line with the trench then through the trench
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author tarini
 */
public class CenterThroughTrench extends Path{

        public CenterThroughTrench(DriveBase driveBase) {
                super(

                        List.of(
                                //starting from center, turns through trench

                                new Pose2d(3.944, -2.628, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.41, -0.706, Rotation2d.fromDegrees(0)), 
                                new Pose2d(7.666, -0.706, Rotation2d.fromDegrees(0))
                        ), 
                driveBase);
                //Logger.consoleLog("running godspeed part 1");
        }
}
