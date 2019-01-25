package frc.team670.robot.subsystems.elbow;

/**
 * Allows for using an Elbow for unit testing
 */
public interface ElbowInterface {

    public void setMotionMagicSetpoint(double wristAngle);

    public void initializeMotionmagic();

    public double getAngle();

}