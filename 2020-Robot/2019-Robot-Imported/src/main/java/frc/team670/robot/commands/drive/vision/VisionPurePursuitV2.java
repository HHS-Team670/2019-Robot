/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.drive.purePursuit.Path;
import frc.team670.robot.commands.drive.purePursuit.PathGenerator;
import frc.team670.robot.commands.drive.purePursuit.PoseEstimator;
import frc.team670.robot.commands.drive.purePursuit.PurePursuitTracker;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.MutableDouble;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.math.DrivePower;
import frc.team670.robot.utils.math.Vector;

/**
 * Essentially a combination of VisionPurePursuit and PurePursuit where everything that needs to be done by VisionPurePursuit for setup
 * is done in the initialize method so this will work properly within a CommandGroup.
 */
public class VisionPurePursuitV2 extends Command {

    private static final double MAX_VEL = 7.5, MAX_ACC = 57, MAX_VELK = 4.5; // VELK = Curve Velocity (1-5)
    private static final double B = 0.9, A = 1 - B, TOLERANCE = 0.001;
    private static final double SPACING = 1; // Spacing Inches

    private static final double LOOKAHEAD_DISTANCE_AT_66_INCHES = 15;

    private PurePursuitTracker purePursuitTracker;
    private PoseEstimator poseEstimator;
    private DriveBase driveBase;
    private MustangSensors sensors;
    private MustangCoprocessor coprocessor;
    private boolean isReversed;
    private int executeCount;
    private MutableDouble finalAngle;
    private double offset;
    private Vector endPoint;

    /**
     * @param finalAngle a MutableDouble object reference to the angle (using zeroed yaw) this PurePursuit command should end up at compared to the zeroed yaw.
     */
    public VisionPurePursuitV2(DriveBase driveBase, MustangSensors sensors, boolean isReversed, MutableDouble finalAngle, double offset, MustangCoprocessor coprocessor) {
        requires(driveBase);
        
        this.driveBase = driveBase;
        this.sensors = sensors;
        this.finalAngle = finalAngle;
        this.coprocessor = coprocessor;
        this.isReversed = isReversed;
        this.offset = offset;
        endPoint = new Vector();

        poseEstimator = new PoseEstimator(driveBase, sensors);
        purePursuitTracker = new PurePursuitTracker(poseEstimator, driveBase, sensors, isReversed);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        try {
            if(MathUtils.doublesEqual(coprocessor.getVisionValues()[2], RobotConstants.VISION_ERROR_CODE)) {
                System.out.println("VPP timestamp"+ SmartDashboard.getNumberArray("reflect_tape_vision_data", new double[]{RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE,RobotConstants.VISION_ERROR_CODE})[2]);
                SmartDashboard.putString("warnings", "Coprocess Camera Unplugged: Vision Down");
                Logger.consoleLog("Coprocess Camera Unplugged: Vision Down");
            return;
            } 
        } catch(IndexOutOfBoundsException e) {
            return;
        }
    
        double horizontalAngle = coprocessor.getAngleToWallTarget();
        System.out.println("HorizontalAngle: " + horizontalAngle);
        if (MathUtils.doublesEqual(horizontalAngle, RobotConstants.VISION_ERROR_CODE)) {
            Logger.consoleLog("No Valid Vision Data found, command quit.");
            SmartDashboard.putString("vision-status", "invalid-data");
            SmartDashboard.putString("warnings", "Vision Target Not Found");
            return;
        }
    
        double ultrasonicDistance;
        if (!isReversed) {
          ultrasonicDistance = sensors.getFrontUltrasonicDistance();
        } else {
          ultrasonicDistance = sensors.getAdjustedBackUltrasonicDistance();
        }
        ultrasonicDistance *= Math.cos(Math.toRadians(horizontalAngle)); // use cosine to get the straight ultrasonic
                                                                         // distance not the diagonal one
    
        double visionDistance = coprocessor.getDistanceToWallTarget();
    
        double straightDistance = visionDistance; // Uses vision distance as a default
        if(ultrasonicDistance < 150 && ultrasonicDistance > 5) {
            if (MathUtils.doublesEqual(visionDistance, RobotConstants.VISION_ERROR_CODE)) {
                straightDistance = ultrasonicDistance;
            } else if (Math.abs(visionDistance-ultrasonicDistance) < 10){ // Checks to make sure both are within a reasonable range of each other and then takes the smaller one
                straightDistance = (visionDistance < ultrasonicDistance) ? visionDistance : ultrasonicDistance;
            }
        }
    
        if (straightDistance > 150) { // Distance is too far, must be invalid data.
            Logger.consoleLog("No Valid Vision Data or Ultrasonic Data found, command quit.");
            SmartDashboard.putString("vision-status", "invalid-data");
            return;
        }
    
        straightDistance = straightDistance - offset;
        if (straightDistance < 0) {
            System.out.println("Too close to target!");
            // this.cancel();
            SmartDashboard.putString("vision-status", "error");
            return;
        }
    
        SmartDashboard.putString("vision-status", "engaged");
    
        System.out.println("Angle: " + coprocessor.getAngleToWallTarget());
        // horizontal distance - when going forward a positive horizontal distance is
        // right and negative is left
        double horizontalDistance = straightDistance * Math.tan(Math.toRadians(horizontalAngle)); // x = y * tan(theta)
        double partialDistanceY = (straightDistance) * 2.0 / 5.0;
    
    
        System.out.println("straightDist: " + straightDistance + ", horizontalDistance: " + horizontalDistance);
        if (isReversed) { // Flip all of these to match our coord system when looking out the back
            straightDistance *= -1;
            horizontalDistance *= -1;
            partialDistanceY *= -1;
        }
    
        sensors.zeroYaw(); // NEEDS to happen
    
        PoseEstimator poseEstimator = new PoseEstimator(driveBase, sensors);
    
        // Make this start with the Pose Estimator and get its position at this poitn for starting coords.
        Vector startPose = poseEstimator.getPose();
        double startX = startPose.x, startY = startPose.y;
        Vector partialDistance = new Vector(startX + horizontalDistance, startY + partialDistanceY);
        endPoint = new Vector(startX + horizontalDistance, startY + (straightDistance));

        PathGenerator generator = new PathGenerator(SPACING);
        generator.setVelocities(MAX_VEL, MAX_ACC, MAX_VELK);
        generator.setSmoothingParameters(A, B, TOLERANCE);
        generator.addPoints(startPose, partialDistance, endPoint); // Right Angle Segment. All of these are negative since we are driving 2018 Robot backwards.
        
        System.out.println(startPose);
        System.out.println(partialDistance);
        System.out.println(endPoint);
        
        Path path = generator.generatePath(isReversed);
    
        finalAngle.setValue(horizontalAngle);
        purePursuitTracker.setPath(path, straightDistance*1/3);
        Robot.leds.setVisionData(true);

        driveBase.initBrakeMode();
        sensors.zeroYaw();
        purePursuitTracker.reset();
        Logger.consoleLog();
        executeCount = 0;

        System.out.println("Start, Pose: " + poseEstimator.getPose());
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        poseEstimator.update();
        DrivePower drivePower;
        drivePower = purePursuitTracker.update(poseEstimator.getPose(), driveBase.getLeftMustangEncoderVelocityInInchesPerSecond(), driveBase.getRightMustangEncoderVelocityInInchesPerSecond(), sensors.getRotationAngle().radians());
    
        driveBase.tankDrive(drivePower.getLeft()/60, drivePower.getRight()/60); //Returns in inches/s
        if(executeCount % 5 == 0){
            // Logger.consoleLog("Powers (inches): leftPower: %s, rightPower: %s, Pose: %s", drivePower.getLeft(), drivePower.getRight(), poseEstimator.getPose());
        }
        executeCount++;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return purePursuitTracker.isDone() || purePursuitTracker.getPath() == null; // sensors.getUltrasonicDistance() < 15;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Vector pose = poseEstimator.getPose();
        // VisionPurePursuit.disableArmRestriction();
       driveBase.stop();
        double xOffset = endPoint.x - pose.x;
        double yOffset = endPoint.y + offset - pose.y;
        double angle = Math.toDegrees(Math.atan(yOffset/xOffset));

        finalAngle.setValue(finalAngle.getValue() - sensors.getYawDouble() - angle);

        Logger.consoleLog("Pose: %s, Pivot angle: %s", pose, angle);
        purePursuitTracker.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
        Logger.consoleLog();
    }
}
