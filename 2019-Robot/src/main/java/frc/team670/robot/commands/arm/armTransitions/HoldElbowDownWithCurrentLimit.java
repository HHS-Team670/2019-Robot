package frc.team670.robot.commands.arm.armTransitions;

import edu.wpi.first.wpilibj.command.*;

import frc.team670.robot.Robot;

public class HoldElbowDownWithCurrentLimit extends Command {
    private static final double elbowOutput = 1.0; // Might have to make this negative depending on how motors are oriented.
    private int currentLimit;

    /**
     * Sets the current limit for the arm
     * @param currentLimit the desired current limit
     */
    public HoldElbowDownWithCurrentLimit(int currentLimit){
        super();
        requires(Robot.arm);
        this.currentLimit = currentLimit;
    }

    /**
     * Enables and sets the current
     */
    @Override
    protected void initialize(){
        Robot.elbow.setCurrentLimit(currentLimit);
        Robot.elbow.enableCurrentLimit();
    }

    /**
     * Moves the arm down at full speed so that the robot can be brought forward onto the platform
     */
    @Override
    protected void execute(){
        Robot.elbow.setOutput(elbowOutput);
    }

    /**
     * Assuming that will never reach isFinished because a command will just interrupt this one
     */
    @Override
    protected boolean isFinished(){
        return false;
    }

    @Override
    protected void end(){
        Robot.elbow.disableCurrentLimit();   
        //Return to default state
    }

    @Override
    protected void interrupted(){
        end();
    }
}