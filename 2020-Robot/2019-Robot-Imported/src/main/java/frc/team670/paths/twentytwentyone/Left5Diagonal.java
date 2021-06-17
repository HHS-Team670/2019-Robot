package frc.team670.paths.twentytwentyone;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;

/**
 * 2021 field
 * trajectory starting to the left of the port (in perspective of the driver)
 * goes under the switch from the left side and intakes 5 balls in a diagonal line, starting closer to the 2 ball line (rather than 3 ball line)
 * BACK of robot starts on initiation line
 * tight bc enters and exists from same top left quadrant of switch
 * google doc link: https://docs.google.com/document/d/1Sm1xBvk9HHvbotRF8uDx5V0EHe8H6kkJBm6AvSKBY1k/edit?usp=sharing
 * @author elisevbp
 */
public class Left5Diagonal extends Path{

        public Left5Diagonal (DriveBase driveBase) {
                super(
                        List.of(
                                //tight equivalent
                                //TODO: CHANGE STARTING POSE SO BACK IS ON INITIATION LINE AND ADJUST PATH
                                // new Pose2d(3.944, -3.915, Rotation2d.fromDegrees(0)),
                                // new Pose2d(5.787, -5.159, Rotation2d.fromDegrees(16.172)),
                                // new Pose2d(5.787, -4.292, Rotation2d.fromDegrees(76.667)),
                                // new Pose2d(6.063, -3.188, Rotation2d.fromDegrees(76.667))

                                //wide equivalent
                                //wide bc enters switch from bottom left quadrant and exists from top left quadrant

                                new Pose2d(3.944, -3.915, Rotation2d.fromDegrees(0)),
                                new Pose2d(5.787, -6.141, Rotation2d.fromDegrees(0)),
                                new Pose2d(6.715, -5.564, Rotation2d.fromDegrees(55.567)),
                                new Pose2d(6.492, -2.575, Rotation2d.fromDegrees(-87.583))
                        ), 
                driveBase);
        }
}
