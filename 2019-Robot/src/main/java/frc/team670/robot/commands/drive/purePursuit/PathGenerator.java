package frc.team670.robot.commands.drive.purePursuit;

import frc.team670.robot.utils.math.Vector;

import java.util.ArrayList;
import java.util.Arrays;

public class PathGenerator {
    private double spacing;
    private double a = 0, b = 0, tolerance = 0;
    private ArrayList<Vector> points = new ArrayList<>();
    private double maxVel, maxAccel, maxVelk;

    public PathGenerator(double spacing) {
        this.spacing = spacing;
    }

    public void setSmoothingParameters(double a, double b, double tolerance) {
        this.a = a;
        this.b = b;
        this.tolerance = tolerance;
    }

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

    public Path generatePath() {
        Path path = new Path(spacing);
        for (int i = 0; i < points.size() - 1; ++i)
            path.addSegment(points.get(i), points.get(i + 1));
        path.addLastPoint();
        if (tolerance != 0)
            path.smooth(a, b, tolerance);
        path.initializePath(maxVel, maxAccel, maxVelk);
        return path;
    }
}
