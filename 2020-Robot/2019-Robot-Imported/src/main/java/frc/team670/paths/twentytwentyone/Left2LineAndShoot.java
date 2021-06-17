package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

public class Left2LineAndShoot extends Path {
    public Left2LineAndShoot(DriveBase driveBase) {
        super(
                List.of(
                        new Pose2d(3.953, -3.915, Rotation2d.fromDegrees(0)),
                        new Pose2d(5.941, -4.227, Rotation2d.fromDegrees(20.271)),
                        new Pose2d(7.667, -3.642, Rotation2d.fromDegrees(22.52)),
                        new Pose2d(5.027, -4.669, Rotation2d.fromDegrees(180 - 22.52))
                ), 
        driveBase);
}
}
