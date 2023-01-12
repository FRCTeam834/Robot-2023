package frc.robot.utility;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

// Taken from:
// https://github.com/Mechanical-Advantage/RobotCode2020/blob/master/src/main/java/frc/robot/util/TunableNumber.java

public class TuneableNumber {
    private NetworkTableEntry entry;
    private double defaultValue;
    private boolean tuningMode;
    private double lastValue;

    /** Create a new TunableNumber */
    public TuneableNumber(NetworkTable table, String name, double defaultValue, boolean tuningMode) {
      this.tuningMode = tuningMode;
      this.entry = table.getEntry(name);
      setDefault(defaultValue);
      lastValue = defaultValue;
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
      if (tuningMode) {
          // This makes sure the data is on NetworkTables but will not change it
          entry.setDouble(defaultValue);
      }
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode
     *
     * @return The current value
     */
    public double get() {
      return get(true);
    }

    private double get(boolean resetLastValue) {
      double currentValue = tuningMode ? entry.getDouble(defaultValue) : defaultValue;
      if (resetLastValue) {
        lastValue = currentValue;
      }
      return currentValue;
    }

    /**
     * Checks if value has changed
     * 
     * @return
     */
    public boolean hasChanged() {
      return lastValue == get(false);
    }
}