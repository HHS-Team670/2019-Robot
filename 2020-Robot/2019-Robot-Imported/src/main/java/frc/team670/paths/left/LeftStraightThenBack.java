package frc.team670.paths.left;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

public class LeftStraightThenBack extends Path {

    public LeftStraightThenBack(DriveBase driveBase) {
            super(
                    List.of(
                        new Pose2d(3.953, -3.915, Rotation2d.fromDegrees(0)),
                        new Pose2d(7.053, -3.915, Rotation2d.fromDegrees(0)), //Forward 2 meters
                        new Pose2d(3.953, -3.915, Rotation2d.fromDegrees(-180))  //backwards 2 meters
                    ), 
            driveBase);
    }
    
}
