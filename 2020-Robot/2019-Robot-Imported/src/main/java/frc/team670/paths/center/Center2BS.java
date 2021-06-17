package frc.team670.paths.center;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.paths.Path;

/**
 * Trajectory starting on the line closest to your score port (robot facing towards your own driverstation), 
 * and facing the 2 Power Cells under the generator near your trench side.
 * 
 * @author meganchoy, ctychen
 */
public class Center2BS extends Path{

        public Center2BS(DriveBase driveBase) {
                super(
                        List.of(
                        new Pose2d(3.8, -2.4, Rotation2d.fromDegrees(0)),
                        new Pose2d(6.105, -2.751, Rotation2d.fromDegrees(-67.5))
                        ),
                driveBase);
        }
}