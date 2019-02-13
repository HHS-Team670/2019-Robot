package frc.team670.robot.commands.drive.purePursuit;

import java.util.ArrayList;
import java.util.Arrays;
import frc.team670.robot.utils.math.Vector;

public class PathGenerator {
    private double spacing;
    private double a = 0, b = 0, tolerance = 0;
    private ArrayList<Vector> points = new ArrayList<>();
    private double maxVel, maxAccel, maxVelk;

    public PathGenerator(double spacing) {
        this.spacing = spacing;
    }

    /**
	 * Smooths the path using gradient descent
	 * @param a 1-b
	 * @param b smoothing factor (higher = more smooth)
	 * @param tolerance convergence tolerance amount (higher = less smoothing)
	 */
    public void setSmoothingParameters(double a, double b, double tolerance) {
        this.a = a;
        this.b = b;
        this.tolerance = tolerance;
    }

    /**
	 * Initializes the path
	 *
	 * @param maxVel   maximum robot velocity
	 * @param maxAccel maximum robot acceleration
	 * @param maxVelk  maximum turning velocity (between 1-5)
	 */
    public void setVelocities(double maxVel, double maxAccel, double maxVelk) {
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxVelk = maxVelk;
    }

    public void addPoint(Vector point) {
        points.add(point);
    }

    public void addPoints(Vector... points) {
        this.points.addAll(Arrays.asList(points));
    }


    /**
     * Generates a not-reversed path
     */
    public Path generatePath() {
        return generatePath(false);
    }
        
    public Path generatePath(boolean reverse) {
        Path path = new Path(spacing, reverse);
        for (int i = 0; i < points.size() - 1; ++i)
            path.addSegment(points.get(i), points.get(i + 1));
        path.addLastPoint();
        if (tolerance != 0)
            path.smooth(a, b, tolerance);
        path.initializePath(maxVel, maxAccel, maxVelk);
        return path;
    }
}
