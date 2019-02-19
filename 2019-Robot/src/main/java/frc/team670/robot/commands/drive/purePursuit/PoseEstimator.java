package frc.team670.robot.commands.drive.purePursuit;

import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.math.Kinematics;
import frc.team670.robot.utils.math.RigidTransform;
import frc.team670.robot.utils.math.Rotation;
import frc.team670.robot.utils.math.Twist;
import frc.team670.robot.utils.math.Vector;

public class PoseEstimator {
	private Twist velocity;
	private RigidTransform pose;
	private RigidTransform prevPose;
	private double prevLeftDist = 0;
	private double prevRightDist = 0;
	private DriveBase driveBase;
	private MustangSensors sensors;

	public PoseEstimator(DriveBase driveBase, MustangSensors sensors) {
		reset();
		this.driveBase = driveBase;
		this.sensors = sensors;

		prevLeftDist = driveBase.getLeftMustangEncoderPositionInInches();
		prevRightDist = driveBase.getRightMustangEncoderPositionInInches();
		update();
		// System.out.println("Start Pose: " + pose);
	}
	public void setDriveTrainBase(DriveBase driveTrainBase) {
		this.driveBase = driveTrainBase;
	}

	public void setSensors(MustangSensors sensors) {
		this.sensors = sensors;
	}

	public Vector getPose() {
		return new Vector(pose.getTranslation().getX(), pose.getTranslation().getY());
	}

	public Twist getVelocity() {
		return velocity;
	}

	/**
	 * MUST BE CALLED IMMEDIATELY AFTER RESETTING DRIVETRAIN/GYRO!
	 */
	public void reset() {
		velocity = new Twist();
		pose = new RigidTransform();
		prevPose = new RigidTransform();
		prevLeftDist = 0;
		prevRightDist = 0;
	}

	public void init() {
		reset();
	}

	public void update() {
		double leftDist, rightDist;
		leftDist = driveBase.getLeftMustangEncoderPositionInInches(); //Not negative because encoders increase when robot moves forward
		rightDist = driveBase.getRightMustangEncoderPositionInInches(); // Right encoder doesnt work
		System.out.println("LeftDist: " + leftDist + ", RightDist: " + rightDist);
		double deltaLeftDist = leftDist - prevLeftDist;
		double deltaRightDist = rightDist - prevRightDist;
		Rotation deltaHeading = prevPose.getRotation().inverse().rotate(sensors.getRotationAngle());

		//Use encoders + gyro to determine our velocity
		velocity = Kinematics.forwardKinematics(deltaLeftDist, deltaRightDist, deltaHeading.radians());

		//use velocity to determine our pose
		pose = Kinematics.integrateForwardKinematics(prevPose, velocity);
		//update for next iteration
		// System.out.println(pose);
		prevLeftDist = leftDist;
		prevRightDist = rightDist;
		prevPose = pose;
	}

	public void end() {

	}
}
