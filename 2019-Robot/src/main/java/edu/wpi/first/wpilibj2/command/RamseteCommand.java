/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj2.command;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.team670.robot.utils.Logger;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;
  private final boolean m_usePID;

  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final double m_ks;
  private final double m_kv;
  private final double m_ka;
  private final DifferentialDriveKinematics m_kinematics;
  private final DoubleSupplier m_leftSpeed;
  private final DoubleSupplier m_rightSpeed;
  private final PIDController m_leftController;
  private final PIDController m_rightController;
  private final BiConsumer<Double, Double> m_output;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * PID control and feedforward are handled internally, and outputs are scaled -1 to 1 for easy
   * consumption by speed controllers.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this
   * is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory                     The trajectory to follow.
   * @param pose                           A function that supplies the robot pose - use one of
   *                                       the odometry classes to provide this.
   * @param controller                       The RAMSETE controller used to follow the trajectory.
   * @param ksVolts                        Constant feedforward term for the robot drive.
   * @param kvVoltSecondsPerMeter          Velocity-proportional feedforward term for the robot
   *                                       drive.
   * @param kaVoltSecondsSquaredPerMeter   Acceleration-proportional feedforward term for the robot
   *                                       drive.
   * @param kinematics                     The kinematics for the robot drivetrain.
   * @param leftWheelSpeedMetersPerSecond  A function that supplies the speed of the left side of
   *                                       the robot drive.
   * @param rightWheelSpeedMetersPerSecond A function that supplies the speed of the right side of
   *                                       the robot drive.
   * @param leftController                 The PIDController for the left side of the robot drive.
   * @param rightController                The PIDController for the right side of the robot drive.
   * @param outputVolts                    A function that consumes the computed left and right
   *                                       outputs (in volts) for the robot drive.
   * @param requirements                   The subsystems to require.
   */
  @SuppressWarnings("PMD.ExcessiveParameterList")
  public RamseteCommand(Trajectory trajectory,
                        Supplier<Pose2d> pose,
                        RamseteController controller,
                        double ksVolts,
                        double kvVoltSecondsPerMeter,
                        double kaVoltSecondsSquaredPerMeter,
                        DifferentialDriveKinematics kinematics,
                        DoubleSupplier leftWheelSpeedMetersPerSecond,
                        DoubleSupplier rightWheelSpeedMetersPerSecond,
                        PIDController leftController,
                        PIDController rightController,
                        BiConsumer<Double, Double> outputVolts,
                        Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    m_follower = requireNonNullParam(controller, "controller", "RamseteCommand");
    m_ks = ksVolts;
    m_kv = kvVoltSecondsPerMeter;
    m_ka = kaVoltSecondsSquaredPerMeter;
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    m_leftSpeed = requireNonNullParam(leftWheelSpeedMetersPerSecond,
                                      "leftWheelSpeedMetersPerSecond",
                                      "RamseteCommand");
    m_rightSpeed = requireNonNullParam(rightWheelSpeedMetersPerSecond,
                                       "rightWheelSpeedMetersPerSecond",
                                       "RamseteCommand");
    m_leftController = requireNonNullParam(leftController, "leftController", "RamseteCommand");
    m_rightController = requireNonNullParam(rightController, "rightController", "RamseteCommand");
    m_output = requireNonNullParam(outputVolts, "outputVolts", "RamseteCommand");

    m_usePID = true;

    //addRequirements(requirements);
  }

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
   * from the RAMSETE controller, and will need to be converted into a usable form by the user.
   *
   * @param trajectory            The trajectory to follow.
   * @param pose                  A function that supplies the robot pose - use one of
   *                              the odometry classes to provide this.
   * @param follower              The RAMSETE follower used to follow the trajectory.
   * @param kinematics            The kinematics for the robot drivetrain.
   * @param outputMetersPerSecond A function that consumes the computed left and right
   *                              wheel speeds.
   * @param requirements          The subsystems to require.
   */
  public RamseteCommand(Trajectory trajectory,
                        Supplier<Pose2d> pose,
                        RamseteController follower,
                        DifferentialDriveKinematics kinematics,
                        BiConsumer<Double, Double> outputMetersPerSecond,
                        Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
    m_pose = requireNonNullParam(pose, "pose", "RamseteCommand");
    m_follower = requireNonNullParam(follower, "follower", "RamseteCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "RamseteCommand");
    m_output = requireNonNullParam(outputMetersPerSecond, "output", "RamseteCommand");

    m_ks = 0;
    m_kv = 0;
    m_ka = 0;
    m_leftSpeed = null;
    m_rightSpeed = null;
    m_leftController = null;
    m_rightController = null;

    m_usePID = false;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_prevTime = 0;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(initialState.velocityMetersPerSecond,
                          0,
                          initialState.curvatureRadPerMeter
                              * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    if (m_usePID) {
      m_leftController.reset();
      m_rightController.reset();
    }
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_usePID) {
      double leftFeedforward =
          m_ks * Math.signum(leftSpeedSetpoint)
              + m_kv * leftSpeedSetpoint
              + m_ka * (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt;

      double rightFeedforward =
          m_ks * Math.signum(rightSpeedSetpoint)
              + m_kv * rightSpeedSetpoint
              + m_ka * (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt;

      leftOutput = leftFeedforward
          + m_leftController.calculate(m_leftSpeed.getAsDouble(),
                                       leftSpeedSetpoint);

      rightOutput = rightFeedforward
          + m_rightController.calculate(m_rightSpeed.getAsDouble(),
                                        rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }
    Logger.consoleLog("LeftspeedSetPoint: %s, Left Speed: %s, LeftOutput: %s, rightSpeedSetPoint: %s Right Speed: %s rightOutput: %s", leftSpeedSetpoint, m_leftSpeed.getAsDouble(), leftOutput, rightSpeedSetpoint, m_rightSpeed.getAsDouble(), rightOutput);
    Logger.consoleLog("Pose: %s", m_pose.get().toString());
    m_output.accept(leftOutput, rightOutput);

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
  }
}