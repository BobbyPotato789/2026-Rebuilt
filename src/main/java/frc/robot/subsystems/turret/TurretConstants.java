package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;

public class TurretConstants {
  public static final int kMotorID = 10;
  public static final int kEncoder1 =
      0; // 18T Pinion Encoder (These are the best tooth pinions I've found)
  public static final int kEncoder2 = 1; // 17T Pinion Encoder

  // CRT Gearing Constants
  public static final int kTurretGearTeeth = 200;
  public static final int kEncoder1Teeth = 18;
  public static final int kEncoder2Teeth = 17;

  // Motor Drive Constants
  // This Gear Ratio should be changed depending on the motor NEO V1.1 wouldn't be strong on it's
  // own
  public static final double kMotorPinionTeeth = 10.0;
  public static final double kMotorGearRatio = kTurretGearTeeth / kMotorPinionTeeth;

  // Constraints
  // Should be changed when Tuning
  public static final double kMaxVelocityRadPerSec = Units.degreesToRadians(180);
  public static final double kMaxAccelRadPerSec = Units.degreesToRadians(360);

  // Limits (Soft Stops)
  // Should also be changed depending on the wire track
  public static final double kMinAngleRad = Units.degreesToRadians(-90);
  public static final double kMaxAngleRad = Units.degreesToRadians(90);
}
