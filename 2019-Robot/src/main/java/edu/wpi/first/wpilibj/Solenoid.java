/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.SolenoidJNI;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Solenoid class for running high voltage Digital Output on the PCM.
 *
 * <p>The Solenoid class is typically used for pneumatic solenoids, but could be used for any
 * device within the current spec of the PCM.
 */
public class Solenoid extends SolenoidBase implements Sendable, AutoCloseable {
  private final int m_channel; // The channel to control.
  private int m_solenoidHandle;

  /**
   * Constructor using the default PCM ID (defaults to 0).
   *
   * @param channel The channel on the PCM to control (0..7).
   */
  public Solenoid(final int channel) {
    this(SensorUtil.getDefaultSolenoidModule(), channel);
  }

  /**
   * Constructor.
   *
   * @param moduleNumber The CAN ID of the PCM the solenoid is attached to.
   * @param channel      The channel on the PCM to control (0..7).
   */
  public Solenoid(final int moduleNumber, final int channel) {
    super(moduleNumber);
    m_channel = channel;

    SensorUtil.checkSolenoidModule(m_moduleNumber);
    SensorUtil.checkSolenoidChannel(m_channel);

    int portHandle = HAL.getPortWithModule((byte) m_moduleNumber, (byte) m_channel);
    m_solenoidHandle = SolenoidJNI.initializeSolenoidPort(portHandle);

    HAL.report(tResourceType.kResourceType_Solenoid, m_channel + 1, m_moduleNumber + 1);
    SendableRegistry.addLW(this, "Solenoid", m_moduleNumber, m_channel);
  }

  @Override
  public void close() {
    SendableRegistry.remove(this);
    SolenoidJNI.freeSolenoidPort(m_solenoidHandle);
    m_solenoidHandle = 0;
  }

  /**
   * Set the value of a solenoid.
   *
   * @param on True will turn the solenoid output on. False will turn the solenoid output off.
   */
  public void set(boolean on) {
    SolenoidJNI.setSolenoid(m_solenoidHandle, on);
  }

  /**
   * Read the current value of the solenoid.
   *
   * @return True if the solenoid output is on or false if the solenoid output is off.
   */
  public boolean get() {
    return SolenoidJNI.getSolenoid(m_solenoidHandle);
  }

  /**
   * Check if solenoid is blacklisted. If a solenoid is shorted, it is added to the blacklist and
   * disabled until power cycle, or until faults are cleared.
   *
   * @return If solenoid is disabled due to short.
   * @see #clearAllPCMStickyFaults()
   */
  public boolean isBlackListed() {
    int value = getPCMSolenoidBlackList() & (1 << m_channel);
    return value != 0;
  }

  /**
   * Set the pulse duration in the PCM. This is used in conjunction with
   * the startPulse method to allow the PCM to control the timing of a pulse.
   * The timing can be controlled in 0.01 second increments.
   *
   * @param durationSeconds The duration of the pulse, from 0.01 to 2.55 seconds.
   *
   * @see #startPulse()
   */
  public void setPulseDuration(double durationSeconds) {
    long durationMS = (long) (durationSeconds * 1000);
    SolenoidJNI.setOneShotDuration(m_solenoidHandle, durationMS);
  }

  /**
   * Trigger the PCM to generate a pulse of the duration set in
   * setPulseDuration.
   *
   * @see #setPulseDuration(double)
   */
  public void startPulse() {
    SolenoidJNI.fireOneShot(m_solenoidHandle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Solenoid");
    builder.setActuator(true);
    builder.setSafeState(() -> set(false));
    builder.addBooleanProperty("Value", this::get, this::set);
  }
}
