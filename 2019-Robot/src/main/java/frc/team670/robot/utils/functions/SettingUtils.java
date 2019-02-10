/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.functions;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import frc.team670.robot.constants.RobotConstants;

/**
 * Contains utility functions for setting values on certain objects.
 */
public class SettingUtils {

  /**
   * Disables controller and releases its resources.
   */
  public static void releaseController(PIDController controller) {
    controller.disable();
  }

  /**
   * Initializes PID for a TalonSRX controller by setting the PID values, min/max output, the feedback device, and the ramp rate.
   * 
   * @param talon The talon for which the control loop is used for
   * @param slot The slot for the PID
   * @param P The proportional constant for PID
   * @param I The integral constant for PID
   * @param D The derivative constant for PID
   * @param F The feed forward constant for PID
   * @param minOutput The minimum output for PID [-1,1]
   * @param maxOutput The maximum ouptut for PID [-1,1]
   * @param feedbackDevice The type of feedback device (probably an encoder) that this motor controller is using
   * @param secondsFromNeutralToFull The ramp rate.
   */
  public static void initTalonPID(TalonSRX talon, int slot, double P, double I, double D, double F,
      double minOutput, double maxOutput, FeedbackDevice feedbackDevice, double secondsFromNeutralToFull) {
    talon.configSelectedFeedbackSensor(feedbackDevice, 0,
        RobotConstants.kTimeoutMs);
        talon.setSensorPhase(true);

    /* set the peak and nominal outputs, 12V means full */
    talon.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    talon.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    talon.configPeakOutputForward(maxOutput, RobotConstants.kTimeoutMs);
    talon.configPeakOutputReverse(minOutput, RobotConstants.kTimeoutMs);
    talon.configClosedloopRamp(secondsFromNeutralToFull, RobotConstants.kTimeoutMs);
    /*
     * set the allowable closed-loop error, Closed-Loop output will be neutral
     * within this range. See Table in Section 17.2.1 for native units per rotation.
     */
    // talon.configAllowableClosedloopError(slot, RobotConstants.ALLOWABLE_PID_ERROR, RobotConstants.kTimeoutMs); /* always servo */
    /* set closed loop gains in slot0 */
    talon.config_kF(slot, F, RobotConstants.kTimeoutMs);
    talon.config_kP(slot, P, RobotConstants.kTimeoutMs);
    talon.config_kI(slot, I, RobotConstants.kTimeoutMs);
    talon.config_kD(slot, D, RobotConstants.kTimeoutMs);
  }

}
