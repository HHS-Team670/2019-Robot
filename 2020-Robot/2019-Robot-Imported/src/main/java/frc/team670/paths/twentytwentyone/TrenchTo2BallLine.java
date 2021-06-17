package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * starts at trench then goes under switch to intake 2 balls in a row
 * fits 2021 field
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author tarini
 */
public class TrenchTo2BallLine extends Path{

        public TrenchTo2BallLine (DriveBase driveBase) {
                super(

                        List.of(
                                //for now j goes forward 
                                new Pose2d(7.666, -0.706, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.559, -2.916, Rotation2d.fromDegrees(-132.149)), //not 100% sure if this is the right heading
                                new Pose2d(6.246, -4.313, Rotation2d.fromDegrees(-159.174)) 
                        ), 
                driveBase);
                //Logger.consoleLog("running godspeed part 1");
        }
}
