/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.BuildAuton;
import frc.team670.robot.commands.arm.BuildArmSequence;


/**
 * Add your docs here.
 */
public class XKeys {

    private NetworkTableInstance instance;
    private NetworkTable table;

    public XKeys() {
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable("SmartDashboard");
        String autonKey = "autonSequence";
        table.addEntryListener(autonKey, (table2, key2, entry, value, flags) -> {
            Scheduler.getInstance().add(new BuildAuton(value.getStringArray(), Robot.arm, false));
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
        String armKey = "armSequence";
        table.addEntryListener(armKey, (table2, key2, entry, value, flags) -> {
            if (value != null) {
                Scheduler.getInstance().add(new BuildArmSequence(value.getStringArray(), Robot.arm));
                table.getEntry("armSequence").setString(null);
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }
}
