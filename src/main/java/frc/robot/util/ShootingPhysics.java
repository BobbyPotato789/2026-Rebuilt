package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
// import frc.robot.Constants.ShooterConstants;

public class ShootingPhysics {

  // Key: Distance (m), Vlaue: Time of Flight (s)
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Key: Distance (m), Value: RPM
  private static final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

  // Key: Distance (m), Value: Hood Angle (Degrees)
  private static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

  static {
    // The More the data you put the more accurate it's going to be
    // But don't put too much

    // Time of flight (this can be tuned by recording in slo-mo)
    timeOfFlightMap.put(null, null);

    // RPM
    rpmMap.put(null, null);

    // Hood Angle
    hoodMap.put(null, null);
  }

  public record ShootSolution(
      Rotation2d robotHeading, // Field-Relative Robot/Turret Angle
      double flywheelRPM, // Adjusted RPM
      double hoodAngleRad, // Hood POsition
      double effectiveDist // This is for debugging
      ) {}

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

    // 1. Get the initial distance
    double currentDistance = targetLocation.getDistance(currentPose.getTranslation());

    // 2. This is going to run the time of fight stuff multiple times for accuracy
    double timeOffFlight = 0.0;
    Translation2d virtualGoalLocation = targetLocation;
    double effectiveDistance = currentDistance;

    // Run 5 passes (this can be changed depending if we want more accuracy)
    for (int i = 0; i < 5; i++) {
      // Use the look up table for the current guess distance
      timeOffFlight = timeOfFlightMap.get(effectiveDistance);

      // Calculate the virtual goal position
      // This matters because if we are moving 4m/s away from the goal, and the shot takes 1 second
      // the goal moves 4 meters further away relative to use during the shot
      double virtualGoalX =
          targetLocation.getX() - (fieldRelativeSpeeds.vxMetersPerSecond * timeOffFlight);
      double virtualGoalY =
          targetLocation.getY() - (fieldRelativeSpeeds.vyMetersPerSecond * timeOffFlight);

      virtualGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

      // Update the distance for the next loop
      effectiveDistance = virtualGoalLocation.getDistance(currentPose.getTranslation());
    }

    // I deleted the past physics that used latency because it took up too much space

    // This handles leading the shot & rotation
    Rotation2d targetHeading = virtualGoalLocation.minus(currentPose.getTranslation()).getAngle();

    // RPM & Hood: This part tells the shooter to shoot harder/higher
    // if we are moving away
    double targetRPM = rpmMap.get(effectiveDistance);
    double targetHoodDeg = hoodMap.get(effectiveDistance);

    return new ShootSolution(
        targetHeading, targetRPM, Units.degreesToRadians(targetHoodDeg), effectiveDistance);
  }

  // ***DISCLAIMER**
  // THESE ARE OLD CODE; it was used to calculate SOTF using Vectors * latency
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
