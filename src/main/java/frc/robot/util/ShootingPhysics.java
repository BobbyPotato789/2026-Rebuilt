package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
// import frc.robot.Constants.ShooterConstants;

public class ShootingPhysics {

  public record ShootSolution(
      Rotation2d azimuth, // Field-Relative Robot/Turret Angle
      double flywheelRPM, // Adjusted RPM
      double hoodAngleRad) {} // Ideal Hood Angle

  /**
   * Calculates the shooting vector based on robot motion. Implements "V_ball/robot = V_ball/ground
   * - V_robot/ground" * @param currentPose Current robot pose (from Odometry/Vision)
   *
   * @param fieldRelativeSpeeds Current robot velocity (Field Relative!)
   * @param targetLocation Location of the goal (Speaker)
   * @return ShotSolution containing the field-relative angle and the necessary shooter speed
   */
  public static ShootSolution calculateShot(
      Pose2d currentPose, ChassisSpeeds fieldRelativeSpeeds, Translation2d targetLocation) {

    // 1. Latency Compensation
    // This is to Project the robot's position forward by the tuned latency factor
    double latencySec = 0.15; //  Tune this: Camera + Network + Motor Lag
    Translation2d robotVelVec =
        new Translation2d(
            fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

    Translation2d futurePos = currentPose.getTranslation().plus(robotVelVec.times(latencySec));

    // 2. Get Target Vector
    Translation2d targetVec = targetLocation.minus(futurePos);
    double dist = targetVec.getNorm();

    // 3. Calculate Ideal Shot (Stationary)
    // We use a lookup table to get the Ideal HORIZONTAL velocity for this distance.
    // NOTE: If your table assumes total RPM, multiply by cos(pitch) here.
    double idealHoodAngle = getIdealHoodAngle(dist);
    double idealHorizontalSpeed = getStationaryHorizontalSpeed(dist);

    // 4. Vector Subtraction
    // V_shot = V_target - V_robot
    // Scale the normalized target vector by the ideal speed to get V_target
    Translation2d idealGroundVel = targetVec.div(dist).times(idealHorizontalSpeed);

    // The subtraction:
    Translation2d shootVec = idealGroundVel.minus(robotVelVec);

    // 5. Extract Outputs
    Rotation2d fieldAzimuth = shootVec.getAngle();
    double requiredHorizontalSpeed = shootVec.getNorm();

    // 6. Convert to Total Speed (If fixed hood)
    // V_total = V_horizontal / cos(theta)
    // If variable hood, you would solve for pitch here instead.
    // double releaseAngleRad = Math.toRadians(ShooterConstants.kFixedPitchDegrees);
    double requiredTotalVelocity = requiredHorizontalSpeed / Math.cos(idealHoodAngle);

    // Convert m/s to RPM (You need to tune this conversion factor)
    // RPM = (Velocity / (2 * PI * radius)) * 60 * GearRatio
    double calculatedRPM = requiredTotalVelocity * 250.0; // Placeholder conversion

    return new ShootSolution(fieldAzimuth, calculatedRPM, idealHoodAngle);
  }

  // Mock Lookup Table: Distance (m) -> Horizontal Velocity (m/s)
  private static double getStationaryHorizontalSpeed(double distance) {
    // Example: 6 m/s base  + linear increase
    return 8.0 + (distance * 1.2);
  }

  // Mock Lockup Table: distance(m) -> Hood Angle (rad)
  private static double getIdealHoodAngle(double dist) {
    // Example: closer = Higher shot (60 deg), Farther = Lower shot (30)
    // Linear Interpolation or Interpolation Double Tree Map recommended here
    double angleDeg = 60.0 - (dist * 5.0);
    return Units.degreesToRadians(Math.max(25.0, Math.min(60.0, angleDeg)));
  }
}
