package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// This is a wrapper for a number that
public class LoggedTunableNumber {
  private final String key;
  private final LoggedNetworkNumber networkNumber; // This object is updated
  private final double defaultValue;
  private double lastValue;

  public LoggedTunableNumber(String dashboardKey, double defaultValue) {
    this.key = dashboardKey;
    this.defaultValue = defaultValue;
    this.lastValue = defaultValue;
    this.networkNumber = new LoggedNetworkNumber(dashboardKey, defaultValue);

    // Optional This is to push to SmartDashboard
    SmartDashboard.putNumber(dashboardKey, defaultValue);
  }

  public double get() {
    return networkNumber.get();
  }

  // this returns true if the value has changed since the last check
  public boolean hasChanged() {
    double current = get();
    if (current != lastValue) {
      lastValue = current;
      return true;
    }
    return false;
  }
}
