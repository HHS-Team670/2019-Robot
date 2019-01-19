package frc.team670.robot.commands.arm.armTransitions;

import edu.wpi.first.wpilibj.command.*;
import frc.team670.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class SetElbowCurrentLimit extends Command{  // TODO Finish isFinished and end methods

    Arm arm = new Arm();

    public SetElbowCurrentLimit(){
        super();
    }

    @Override
    protected void initialize(){
        arm.setElbowCurrentLimit(10, true);
    }

    @Override
    protected void execute(){
        arm.setElbowAtFullPower();
    }

    @Override
    protected boolean isFinished(){

        return false;
    }

    @Override
    protected void end(){

    }

    @Override
    protected void interrupted(){
        end();
    }
}