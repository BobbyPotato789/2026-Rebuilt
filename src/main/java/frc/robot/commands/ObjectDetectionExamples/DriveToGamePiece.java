package frc.robot.commands.ObjectDetectionExamples;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.LoggedTunableNumber;

public class DriveToGamePiece extends Command {
  private final Drive drive;
  private final Vision vision;

  // Camera Index for the Intake Camera (this depends on how you have it setup on PhotonVision GUI)
  private final int cameraIndex = 0;

  // Tunable PID for steering
  private static final LoggedTunableNumber steerKp =
      new LoggedTunableNumber("Tuning/DriveToGP/SteerkP", 0.05);
  private static final LoggedTunableNumber steerKi =
      new LoggedTunableNumber("Tuning/DriveToGP/SteerkI", 0.0);
  private static final LoggedTunableNumber steerKd =
      new LoggedTunableNumber("Tuning/DriveToGP/SteerkD", 0.0);

  // Forward speed (also tunable) in m/s
  // private static final LoggedTunableNumber forwardSpeed = new
  // LoggedTunableNumber("Tuning/DriveToGP/ForwardSpeed", 1.5);

  // Acceleration Limit (higher = snappier, lower = smoother)
  private static final LoggedTunableNumber maxAccel =
      new LoggedTunableNumber("Turning/DriveToGP/MaxAccel", 3.0);

  // Speed curve (look up table)
  // The Key = Area (%), Value = Speed(m/s)
  private static final InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();

  // The Curve is made from the points you put here
  static {
    // Smaller (%) smaller the Area = the faster the robot should move
    // This can also be used to set the Area it must see in order to move
    speedMap.put(0.5, 3.5);

    speedMap.put(2.0, 3.0);

    // The lowest value should be the Minimum Intake Speed
    speedMap.put(4.0, 1.0);

    speedMap.put(10.0, 1.0);
  }
  // Remember that the points are dependent on Camera placement as well (Tune them)

  private final ProfiledPIDController turnController;
  private final SlewRateLimiter speedLimiter;
  private boolean hasSeenPiece = false;

  public DriveToGamePiece(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    addRequirements(drive);

    // setupd the PID Controller for steering
    turnController =
        new ProfiledPIDController(
            steerKp.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(360), // Max Turn Velocity
                Units.degreesToRadians(720) // Max Turn Acceleration
                ));
    // This will center the GP to the camera, this should be changed if it's angled
    turnController.setGoal(0.0);

    // Setup Speed Limiter
    speedLimiter = new SlewRateLimiter(maxAccel.get());
  }

  @Override
  public void initialize() {
    hasSeenPiece = false;

    // Update PID from Advantage Scope
    turnController.setPID(steerKp.get(), steerKi.get(), steerKd.get());

    // Reset the SlewRateLimiter to the robot's CURRENT speed
    // This Prevents any "jumping" if the robot is moving
    double currentSpeed =
        Math.hypot(
            drive.getChassisSpeeds().vxMetersPerSecond, drive.getChassisSpeeds().vyMetersPerSecond);
    speedLimiter.reset(currentSpeed);
  }

  @Override
  public void execute() {
    // Update tunables if changed
    if (steerKp.hasChanged() || steerKi.hasChanged() || steerKd.hasChanged()) {
      turnController.setPID(steerKp.get(), steerKi.get(), steerKd.get());
    }

    double rotationOutput = 0.0;
    double targetSpeed = 0.0;

    // Check if it sees a game piece
    if (vision.hasGamePiece(cameraIndex)) {
      hasSeenPiece = true;

      // 1. Calculate steering
      // Read the yaw (error) from Vision
      // Goal is 0
      double currentYawRad = Units.degreesToRadians(vision.getGamePieceYaw(cameraIndex));

      rotationOutput = turnController.calculate(currentYawRad);

      // 2. Calculate Speed from the Curve
      double area = vision.getGamePieceArea(cameraIndex);

      // 3. Map: Area -> Speed
      targetSpeed = speedMap.get(area);

      // drive.runVelocity(new ChassisSpeeds(xSpeed, 0.0, rotationOutput));
    } else {
      // This is optional (we can make it scan if it lost it)
      // This should Decelerate
      rotationOutput = 0.0;
      targetSpeed = 0.0;
    }

    // Smooth the Speed
    // This should calculate the next speed allowed by the Max Accel Limit
    double smoothedSpeed = speedLimiter.calculate(targetSpeed);

    // Drive (Should be Robot Relative)
    drive.runVelocity(new ChassisSpeeds(smoothedSpeed, 0.0, rotationOutput));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    // This really depends on how you guys wanna do Auton
    return false;
  }
}
