
  // Called repeatedly when this Command is scheduled to run
  // get point that youre chasing , keep track of current location , compare that to target point and 
  //adjust your angle and speed while driving (estimate location)
  // each time need to update estimate of location
  // update set points (setSetPoint() during execute)

package frc.team670.robot.commands.auto;

import frc.team670.robot.constants.*;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.Robot;
import frc.team254.lib.util.control.*;
import frc.team254.lib.util.motion.*;
import frc.team254.lib.util.math.RigidTransform2d;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.utils.Logger;

public class PurePursuitCommand extends Command
{
    private DriveBase tank;
    private PathFollower pathFollower;
    private AdaptivePurePursuitController pathControl;
    private Path path;
    private Lookahead lookahead;
    private double stopDistance;
    private RigidTransform2d pose;
    private boolean drift;
    private boolean autoFirstPoint;
    private boolean forward;

    public PurePursuitCommand(Path path)
    {
        this(path, RobotConstants.lookahead, RobotConstants.segmentCompletionTolerance);
    }


    public PurePursuitCommand(Path path, Lookahead lookahead, double stopDistance){
        requires(Robot.driveBase);
        this.path = path;
        this.lookahead = lookahead;
        this.stopDistance = stopDistance;
        //something is definitely missing here

    }

    @Override
    protected void initialize()
    {
        Logger.consoleLog("Initialized P.P.D");
        pathControl= new AdaptivePurePursuitController(path, false, lookahead);

    }

    protected void execute()
    {
        pathControl.update(pose);

        //TODO:get location used

        Vector2D wheelVelocities = new Vector2D(0, 0);//TODO:get velocities somehow

        double wheelL = wheelVelocities.getX();
        double wheelR = wheelVelocities.getY();

        Robot.driveBase.tankDrive(wheelL, wheelR, false);

    }

    protected boolean isFinished(){
        boolean finishedPath = pathControl.isFinished();
        if(finishedPath)
        {
            Logger.consoleLog("finished");
            Robot.driveBase.stop();
        }
        return finishedPath;
    }

}