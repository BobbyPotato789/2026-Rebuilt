package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // --- Primary Set (Bottom) ---
    public double primaryLeaderRPM = 0.0;
    public double primaryLeaderVolts = 0.0;
    public double[] primaryLeaderAmps = new double[] {};

    // Primary Follower Data
    public double primaryFollowerVolts = 0.0;
    public double[] primaryFollowerAmps = new double[] {};

    // --- Secondary Set (Top) ---
    public double secondaryLeaderRPM = 0.0;
    public double secondaryLeaderVolts = 0.0;
    public double[] secondaryLeaderAmps = new double[] {};

    // Secondary Follower Data
    public double secondaryFollowerVolts = 0.0;
    public double[] secondaryFollowerAmps = new double[] {};

    // --- Hoods ---
    public double primaryHoodRad = 0.0;
    public double primaryHoodVolts = 0.0;

    public double secondaryHoodRad = 0.0;
    public double secondaryHoodVolts = 0.0;

    public boolean hoodsConnected = false;
  }

  // Updates the set of loggable inputs
  public default void updateInputs(ShooterIOInputs inputs) {}

  // Run the shooters (Volts)
  public default void setPrimaryVolts(double volts) {}

  public default void setSecondaryVolts(double volts) {}

  // Run the Adjustable Hoods (Volts)
  public default void setPrimaryHoodVolts(double volts) {}

  public default void setSecondaryHoodVolts(double volts) {}

  // Configure PID Constants (Optional If tuning Via Advantage Scope)
  public default void configurePID(double kP, double kI, double kD) {}
}
