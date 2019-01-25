package frc.team670.robot.subsystems.wrist;

public interface WristInterface {

    public void setMotionMagicSetpoint(double wristAngle);

    public void initializeMotionmagic();

    public double getAngle();
}