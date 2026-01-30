// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.8;
  }

  public static final class ShooterConstants {
    public static final double kFixedPitchDegrees = 0;

    // --- Configuration Flags ---
    public static final boolean kHasHood = true;
    public static final boolean kIsDoubleFlywheel = true; // Set TRUE for Top/Bottom shooter
    public static final boolean kHasDualHoods = true;

    // --- Device IDs ---
    // Primary Flywheel (e.g. Bottom Roller) - 2 Motors
    public static final int kPrimaryLeaderID = 20;
    public static final int kPrimaryFollowerID = 21;

    // Secondary Flywheel (e.g. Top Roller) - 2 Motors
    public static final int kSecondaryLeaderID = 24;
    public static final int kSecondaryFollowerID = 25;

    // Hoods
    public static final int kPrimaryHoodID = 22;
    public static final int kSecondaryHoodID = 23;

    // --- Flywheel Constants (Bang-Bang) ---
    public static final double kFlywheelToleranceRPM = 50.0;
    public static final double kMaxRPM = 5700.0;
    public static final double kBangBangVoltage = 12.0;

    // --- Hood Constants ---
    public static final double kHoodGearRatio = 100.0;
    public static final double kHoodMinAngleRad = Units.degreesToRadians(20);
    public static final double kHoodMaxAngleRad = Units.degreesToRadians(60);

    // Hood PID
    public static final double kHoodkP = 4.0;
    public static final double kHoodkI = 0.0;
    public static final double kHoodkD = 0.0;

    public static final double kHoodMaxVel = Units.degreesToRadians(120);
    public static final double kHoodMaxAccel = Units.degreesToRadians(240);
  }
}
