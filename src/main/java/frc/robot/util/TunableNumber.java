package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class TunableNumber {
  private static final String tableKey = "TunableNumbers";

  private String key;
  private double defaultValue;
  private double lastHasChangedValue = defaultValue;

  /**
   * Create a new TunableNumber
   * 
   * @param dashboardKey Key on dashboard
   */
  public TunableNumber(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  /**
   * Get the default value for the number that has been set
   * 
   * @return The default value
   */
  public double getDefault() {
    return defaultValue;
  }

  /**
   * Set the default value of the number
   * 
   * @param defaultValue The default value
   */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
    if (Constants.tuningMode) {
      // This makes sure the data is on NetworkTables but will not change it
      SmartDashboard.putNumber(key,
          SmartDashboard.getNumber(key, defaultValue));
    } else {
      SmartDashboard.delete(key);
    }
  }

  /**
   * Get the current value, from dashboard if available and in tuning mode
   * 
   * @return The current value
   */
  public double get() {
    return Constants.tuningMode ? SmartDashboard.getNumber(key, defaultValue)
        : defaultValue;
  }

  /**
   * Checks whether the number has changed since our last check
   * 
   * @return True if the number has changed since the last time this method was called, false
   *         otherwise
   */
  public boolean hasChanged() {
    double currentValue = get();
    if (currentValue != lastHasChangedValue) {
      lastHasChangedValue = currentValue;
      return true;
    }

    return false;
  }
}
