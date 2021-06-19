/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.shuffleboard;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A generic component in Shuffleboard.
 *
 * @param <C> the self type
 */
public abstract class ShuffleboardComponent<C extends ShuffleboardComponent<C>>
    implements ShuffleboardValue {
  private final ShuffleboardContainer m_parent;
  private final String m_title;
  private String m_type;
  private Map<String, Object> m_properties;
  private boolean m_metadataDirty = true;
  private int m_column = -1;
  private int m_row = -1;
  private int m_width = -1;
  private int m_height = -1;

  protected ShuffleboardComponent(ShuffleboardContainer parent, String title, String type) {
    m_parent = requireNonNullParam(parent, "parent", "ShuffleboardComponent");
    m_title = requireNonNullParam(title, "title", "ShuffleboardComponent");
    m_type = type;
  }

  protected ShuffleboardComponent(ShuffleboardContainer parent, String title) {
    this(parent, title, null);
  }

  public final ShuffleboardContainer getParent() {
    return m_parent;
  }

  protected final void setType(String type) {
    m_type = type;
    m_metadataDirty = true;
  }

  public final String getType() {
    return m_type;
  }

  @Override
  public final String getTitle() {
    return m_title;
  }

  /**
   * Gets the custom properties for this component. May be null.
   */
  final Map<String, Object> getProperties() {
    return m_properties;
  }

  /**
   * Sets custom properties for this component. Property names are case- and whitespace-insensitive
   * (capitalization and spaces do not matter).
   *
   * @param properties the properties for this component
   * @return this component
   */
  public final C withProperties(Map<String, Object> properties) {
    m_properties = properties;
    m_metadataDirty = true;
    return (C) this;
  }

  /**
   * Sets the position of this component in the tab. This has no effect if this component is inside
   * a layout.
   *
   * <p>If the position of a single component is set, it is recommended to set the positions of
   * <i>all</i> components inside a tab to prevent Shuffleboard from automatically placing another
   * component there before the one with the specific position is sent.
   *
   * @param columnIndex the column in the tab to place this component
   * @param rowIndex    the row in the tab to place this component
   * @return this component
   */
  public final C withPosition(int columnIndex, int rowIndex) {
    m_column = columnIndex;
    m_row = rowIndex;
    m_metadataDirty = true;
    return (C) this;
  }

  /**
   * Sets the size of this component in the tab. This has no effect if this component is inside a
   * layout.
   *
   * @param width  how many columns wide the component should be
   * @param height how many rows high the component should be
   * @return this component
   */
  public final C withSize(int width, int height) {
    m_width = width;
    m_height = height;
    m_metadataDirty = true;
    return (C) this;
  }

  protected final void buildMetadata(NetworkTable metaTable) {
    if (!m_metadataDirty) {
      return;
    }
    // Component type
    if (getType() == null) {
      metaTable.getEntry("PreferredComponent").delete();
    } else {
      metaTable.getEntry("PreferredComponent").forceSetString(getType());
    }

    // Tile size
    if (m_width <= 0 || m_height <= 0) {
      metaTable.getEntry("Size").delete();
    } else {
      metaTable.getEntry("Size").setDoubleArray(new double[]{m_width, m_height});
    }

    // Tile position
    if (m_column < 0 || m_row < 0) {
      metaTable.getEntry("Position").delete();
    } else {
      metaTable.getEntry("Position").setDoubleArray(new double[]{m_column, m_row});
    }

    // Custom properties
    if (getProperties() != null) {
      NetworkTable propTable = metaTable.getSubTable("Properties");
      getProperties().forEach((name, value) -> propTable.getEntry(name).setValue(value));
    }
    m_metadataDirty = false;
  }

}
