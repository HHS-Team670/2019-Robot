package frc.team670.robot.commands.drive.purePursuit;

import java.util.ArrayList;
import java.util.Optional;

import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.math.DrivePower;
import frc.team670.robot.utils.math.Vector;

/**
 * Given a path and a couple parameters, this class will handle much of the processing of the Pure Pursuit algorithm
 */
public class PurePursuitTracker {

	private static final double FEEDBACK_MULTIPLIER = 0.3;

	private PoseEstimator poseEstimator;
	private int lastClosestPoint;
	private Path path;
	private double lookaheadDistance;
	private double robotTrack = 0;
	// private	DrivePower drivePower;

	// private Notifier updater;

	private boolean isReversed;

	public PurePursuitTracker(PoseEstimator poseEstimator, DriveBase driveBase, MustangSensors sensors, boolean isReversed) {
		this.poseEstimator = poseEstimator;
		this.isReversed = isReversed;
		// updater = new Notifier( new Runnable() {
		// 	public void run() {
		// 		poseEstimator.update();
		// 		drivePower = update(poseEstimator.getPose(), driveBase.getLeftMustangEncoderVelocityInInchesPerSecond(), driveBase.getRightMustangEncoderVelocityInInchesPerSecond(), sensors.getRotationAngle().radians());
		// 	}
		// });
        reset();
	}


	/**
	 * Makes a Pure Pursuit Tracker that is not reversed
	 */
	public PurePursuitTracker(PoseEstimator poseEstimator, DriveBase driveBase, MustangSensors sensors) {
		this(poseEstimator, driveBase, sensors, false);
	}

	// public void startNotifier(double period){
	// 	updater.startPeriodic(period);
	// }

	// public void stopNotifier(){
	// 	updater.stop();
	// }


	/**
	 * Sets the path to be tracked
	 *
	 * @param path              path to be tracked
	 * @param lookaheadDistance lookahead distance (ideally between 15-24 inches)
	 */
	public void setPath(Path path, double lookaheadDistance) {
		this.path = path;
		this.lookaheadDistance = lookaheadDistance;
		robotTrack = RobotConstants.WHEEL_BASE;
	}

	public Path getPath(){
		return path;
	}

	public void reset() {
		this.lastClosestPoint = 0;
	}

	/**
	 * Updates the tracker and returns left and right velocities
	 *
	 * @param currPose current position of robot
	 * @param currLeftVel current left velocity of robot
	 * @param currRightVel current right velocity of robot
	 * @param heading current angle of robot
	 * @return left and right velocities to be sent to drivetrain
	 */
	public DrivePower update(Vector currPose, double currLeftVel, double currRightVel, double heading) {
		boolean onLastSegment = false;
		int closestPointIndex = getClosestPointIndex(currPose);
		Vector lookaheadPoint = new Vector(0, 0);
		if(path == null){
			return new DrivePower(0,0);
		}
		ArrayList<Vector> robotPath = path.getRobotPath();
		for (int i = closestPointIndex + 1; i < robotPath.size(); i++) {
			Vector startPoint = robotPath.get(i - 1);
			Vector endPoint = robotPath.get(i);
			// System.out.println("StartPoint: " + startPoint + ", EndPoint: " + endPoint);
			if (i == robotPath.size() - 1)
				onLastSegment = true;
			Optional<Vector> lookaheadPtOptional = calculateLookAheadPoint(startPoint, endPoint, currPose, lookaheadDistance, onLastSegment);
			if (lookaheadPtOptional.isPresent()) {
				lookaheadPoint = lookaheadPtOptional.get();
				break;
			}
		}
		
		double targetVel = robotPath.get(getClosestPointIndex(currPose)).getVelocity();

		double curvature = path.calculateCurvatureLookAheadArc(currPose, heading, lookaheadPoint, lookaheadDistance);
		double leftTargetVel = calculateLeftTargetVelocity(targetVel, curvature);
		double rightTargetVel = calculateRightTargetVelocity(targetVel, curvature);

		double leftFeedback = FEEDBACK_MULTIPLIER * (leftTargetVel - currLeftVel);
		double rightFeedback = FEEDBACK_MULTIPLIER * (rightTargetVel - currRightVel);

		double leftVel = leftTargetVel + leftFeedback;
		double rightVel = rightTargetVel + rightFeedback;

		// Flips the left and right velocities if it needs to be reversed
		return new DrivePower((isReversed ? rightVel : leftVel), (isReversed ? leftVel : rightVel));
	}

	/**
	 * Calculates the left target velocity given target overall velocity
	 *
	 * @param targetRobotVelocity target overall robot velocity
	 * @param curvature           curvature of path at current point
	 * @return left target velocity
	 */
	private double calculateLeftTargetVelocity(double targetRobotVelocity, double curvature) {
		return targetRobotVelocity * ((2 + (robotTrack * curvature))) / 2;
	}

	/**
	 * Calculates the right target velocity given target overall velocity
	 * @param targetRobotVelocity target overall robot velocity
	 * @param curvature curvature of path at current point
	 * @return right target velocity
	 */
	private double calculateRightTargetVelocity(double targetRobotVelocity, double curvature) {
		return targetRobotVelocity * ((2 - (robotTrack * curvature))) / 2;
	}

	/**
	 * Calculates the intersection t-value between a line and a circle, using quadratic formula
	 *
	 * @param startPoint        start of line
	 * @param endPoint          end of line
	 * @param currPos           current robot position (or center of the circle)
	 * @param lookaheadDistance lookahead distance along the path (or radius of the circle)
	 * @return intersection t-value (scaled from 0-1, representing proportionally how far along the segment the intersection is)
	 */
	private Optional<Double> calcIntersectionTVal(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance) {

		Vector d = Vector.sub(endPoint, startPoint);
		Vector f = Vector.sub(startPoint, currPos);

		double a = d.dot(d);
		double b = 2 * f.dot(d);
		double c = f.dot(f) - Math.pow(lookaheadDistance, 2);
		double discriminant = Math.pow(b, 2) - (4 * a * c);

		if (discriminant < 0) {
			return Optional.empty();
		} else {
			discriminant = Math.sqrt(discriminant);
			double t1 = (-b - discriminant) / (2 * a);
			double t2 = (-b + discriminant) / (2 * a);

			if (t1 >= 0 && t1 <= 1) {
				return Optional.of(t1);
			}
			if (t2 >= 0 && t2 <= 1) {
				return Optional.of(t2);
			}

		}

		return Optional.empty();
	}

	/**
	 * Uses the calculated intersection t-value to get a point on the path of where to look ahead
	 *
	 * @param startPoint        starting point
	 * @param endPoint          ending point
	 * @param currPos           current robot position
	 * @param lookaheadDistance lookahead distance along the path
	 * @param onLastSegment     whether or not we are on the last path segment
	 * @return lookahead point
	 */
	private Optional<Vector> calculateLookAheadPoint(Vector startPoint, Vector endPoint, Vector currPos, double lookaheadDistance, boolean onLastSegment) {
		Optional<Double> tIntersect = calcIntersectionTVal(startPoint, endPoint, currPos, lookaheadDistance);
		if (!tIntersect.isPresent() && onLastSegment) {
			return Optional.of(path.getRobotPath().get(path.getRobotPath().size() - 1));
		} else if (!tIntersect.isPresent()) {
			return Optional.empty();
		} else {
			Vector intersectVector = Vector.sub(endPoint, startPoint, null);
			Vector vectorSegment = Vector.mult(intersectVector, tIntersect.get());
			Vector point = Vector.add(startPoint, vectorSegment);
			return Optional.of(point);
		}
	}

	/**
	 * Calculates the index of the point on the path that is closest to the robot. Also avoids going backwards
	 * @param currPos current robot position
	 * @return index of closest point on path
	 */
	private int getClosestPointIndex(Vector currPos) {
		double shortestDistance = Double.MAX_VALUE;
		int closestPoint = 0;
		if(path == null){
			return 0;
		}
		ArrayList<Vector> robotPath = path.getRobotPath();
		for (int i = lastClosestPoint; i < robotPath.size(); i++) {
			if (Vector.dist(robotPath.get(i), currPos) < shortestDistance) {
				closestPoint = i;
				shortestDistance = Vector.dist(robotPath.get(i), currPos);
			}
		}
		lastClosestPoint = closestPoint;
		return closestPoint;
	}

	/**
	 * Tells PurePursuitAction when we are done, indicated by the closest point being the last point of the path
	 * @return whether or not we should finish
	 */
	public boolean isDone() {
		if(path == null){
			return true;
		}
		return getClosestPointIndex(poseEstimator.getPose()) == path.getRobotPath().size() - 1;
	}
}