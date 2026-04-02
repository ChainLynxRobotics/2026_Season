package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableBoolean {
  private static final String tableKey = "TunableBooleans";

  private String key;
  private boolean defaultValue;
  private boolean lastHasChangedValue = defaultValue;

  /**
   * Create a new TunableBoolean
   *
   * @param dashboardKey Key on dashboard
   */
  public TunableBoolean(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Create a new TunableBoolean with the default value
   *
   * @param dashboardKey Key on dashboard
   * @param defaultValue Default value
   */
  public TunableBoolean(String dashboardKey, Boolean defaultValue) {
    this(dashboardKey);
    setDefault(defaultValue);
  }

  /**
   * Get the default value for the number that has been set
   *
   * @return The default value
   */
  public boolean getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   *
   * @param defaultValue The default value
   */
  public void setDefault(boolean defaultValue) {
    this.defaultValue = defaultValue;
    if (Constants.tuningMode) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   *
   * @return The current value
   */
  public boolean get() {
    return Constants.tuningMode ? SmartDashboard.getBoolean(key, defaultValue) : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   *
   * @return True if the number has changed since the last time this method was called, false
   *     otherwise
   */
  public boolean hasChanged() {
    boolean currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }
}
