/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.shuffleboard;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Consumer;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

final class ShuffleboardInstance implements ShuffleboardRoot {
  private final Map<String, ShuffleboardTab> m_tabs = new LinkedHashMap<>();

  private boolean m_tabsChanged = false; // NOPMD redundant field initializer
  private final NetworkTable m_rootTable;
  private final NetworkTable m_rootMetaTable;
  private final NetworkTableEntry m_selectedTabEntry;

  /**
   * Creates a new Shuffleboard instance.
   *
   * @param ntInstance the NetworkTables instance to use
   */
  ShuffleboardInstance(NetworkTableInstance ntInstance) {
    requireNonNullParam(ntInstance, "ntInstance", "ShuffleboardInstance");
    m_rootTable = ntInstance.getTable(Shuffleboard.kBaseTableName);
    m_rootMetaTable = m_rootTable.getSubTable(".metadata");
    m_selectedTabEntry = m_rootMetaTable.getEntry("Selected");
    HAL.report(tResourceType.kResourceType_Shuffleboard, 0);
  }

  @Override
  public ShuffleboardTab getTab(String title) {
    requireNonNullParam(title, "title", "getTab");
    if (!m_tabs.containsKey(title)) {
      m_tabs.put(title, new ShuffleboardTab(this, title));
      m_tabsChanged = true;
    }
    return m_tabs.get(title);
  }

  @Override
  public void update() {
    if (m_tabsChanged) {
      String[] tabTitles = m_tabs.values()
          .stream()
          .map(ShuffleboardTab::getTitle)
          .toArray(String[]::new);
      m_rootMetaTable.getEntry("Tabs").forceSetStringArray(tabTitles);
      m_tabsChanged = false;
    }
    for (ShuffleboardTab tab : m_tabs.values()) {
      String title = tab.getTitle();
      tab.buildInto(m_rootTable, m_rootMetaTable.getSubTable(title));
    }
  }

  @Override
  public void enableActuatorWidgets() {
    applyToAllComplexWidgets(ComplexWidget::enableIfActuator);
  }

  @Override
  public void disableActuatorWidgets() {
    applyToAllComplexWidgets(ComplexWidget::disableIfActuator);
  }

  @Override
  public void selectTab(int index) {
    m_selectedTabEntry.forceSetDouble(index);
  }

  @Override
  public void selectTab(String title) {
    m_selectedTabEntry.forceSetString(title);
  }

  /**
   * Applies the function {@code func} to all complex widgets in this root, regardless of how they
   * are nested.
   *
   * @param func the function to apply to all complex widgets
   */
  private void applyToAllComplexWidgets(Consumer<ComplexWidget> func) {
    for (ShuffleboardTab tab : m_tabs.values()) {
      apply(tab, func);
    }
  }

  /**
   * Applies the function {@code func} to all complex widgets in {@code container}. Helper method
   * for {@link #applyToAllComplexWidgets}.
   */
  private void apply(ShuffleboardContainer container, Consumer<ComplexWidget> func) {
    for (ShuffleboardComponent<?> component : container.getComponents()) {
      if (component instanceof ComplexWidget) {
        func.accept((ComplexWidget) component);
      }
      if (component instanceof ShuffleboardContainer) {
        apply((ShuffleboardContainer) component, func);
      }
    }
  }
}
