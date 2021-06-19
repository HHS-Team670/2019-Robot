/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

/**
 * Mindsensors SD540 Speed Controller.
 *
 * <p>Note that the SD540 uses the following bounds for PWM values. These values should work
 * reasonably well for most controllers, but if users experience issues such as asymmetric
 * behavior around the deadband or inability to saturate the controller in either direction,
 * calibration is recommended. The calibration procedure can be found in the SD540 User Manual
 * available from Mindsensors.
 *
 * <p><ul>
 * <li>2.05ms = full "forward"
 * <li>1.55ms = the "high end" of the deadband range
 * <li>1.50ms = center of the deadband range (off)
 * <li>1.44ms = the "low end" of the deadband range
 * <li>0.94ms = full "reverse"
 * </ul>
 */
public class SD540 extends PWMSpeedController {
  /**
   * Common initialization code called by all constructors.
   */
  protected void initSD540() {
    setBounds(2.05, 1.55, 1.50, 1.44, 0.94);
    setPeriodMultiplier(PeriodMultiplier.k1X);
    setSpeed(0.0);
    setZeroLatch();

    HAL.report(tResourceType.kResourceType_MindsensorsSD540, getChannel() + 1);
    SendableRegistry.setName(this, "SD540", getChannel());
  }

  /**
   * Constructor.
   *
   * @param channel The PWM channel that the SD540 is attached to. 0-9 are on-board, 10-19 are on
   *                the MXP port
   */
  public SD540(final int channel) {
    super(channel);
    initSD540();
  }
}
