package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};

    // EasyCRT Raw Inputs
    public double encoder1Rotations = 0.0;
    public double encoder2Rotations = 0.0;
    public boolean encodersConnected = true;
  }

  default void updateInputs(TurretIOInputs inputs) {}

  default void setVoltage(double volts) {}

  // This is used to seed the interal SparkMax Encoder of the Turret Motor
  default void setInternalPosition(double positionRad) {}

  // configure the SparkMax PID (if running on controller)
  default void configurePID(double kP, double kI, double kD) {}
}
