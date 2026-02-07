package frc.robot.commands.ShooterExamples;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ShootingPhysics;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveAimSOTF extends Command {
  private final Drive drive;
  private final Shooter shooter;
  private final Supplier<Translation2d> targetSupplier;
  private final DoubleSupplier xInput;
  private final DoubleSupplier yInput;

  private final ProfiledPIDController turnController;

  // Tunable Numbers
  private static final LoggedTunableNumber turnKp =
      new LoggedTunableNumber("Tuning/DriveAim/kP", 5.0);
  private static final LoggedTunableNumber turnKi =
      new LoggedTunableNumber("Tuning/DriveAim/kI", 0.0);
  private static final LoggedTunableNumber turnKd =
      new LoggedTunableNumber("Tuning/DriveAim/kD", 0.0);

  // This is atunable Max Speed Limiter (saftey feature, if your confident go crazy)
  // Remember max MAGNITUDE is 4.8m/s
  private static final LoggedTunableNumber speedLimit =
      new LoggedTunableNumber("Tuning/DriveAim/MaxSpeed", 4.5);

  public DriveAimSOTF(
      Drive drive,
      Shooter shooter,
      Supplier<Translation2d> target,
      DoubleSupplier xInput,
      DoubleSupplier yInput) {

    this.drive = drive;
    this.shooter = shooter;
    this.targetSupplier = target;
    this.xInput = xInput;
    this.yInput = yInput;

    addRequirements(drive, shooter);

    // Tune for your Drivetrain mass/friction
    turnController =
        new ProfiledPIDController(
            5.0,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(540), Units.degreesToRadians(720)));
    turnController.setPID(turnKp.get(), turnKi.get(), turnKd.get());
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void execute() {
    // Update the Tunables
    if (turnKp.hasChanged() || turnKi.hasChanged() || turnKd.hasChanged()) {
      turnController.setPID(turnKp.get(), turnKi.get(), turnKd.get());
    }
    // 1. Get  Inputs
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fielSpeeds = drive.getFieldRelativeSpeeds();

    // 2. Calculate SOTF Solution
    var solution = ShootingPhysics.calculateShot(robotPose, fielSpeeds, targetSupplier.get());

    // 3. Calculate Turn Output
    // We want the Robot's Heading (Odometry) to match the Shot Azimuth
    double rotationOutput =
        turnController.calculate(
            robotPose.getRotation().getRadians(), solution.robotHeading().getRadians());

    // 4. Drive
    // Pass Driver X/Y, but override Rotation with SOTF result
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xInput.getAsDouble() * speedLimit.get(),
            yInput.getAsDouble() * speedLimit.get(),
            rotationOutput,
            drive.getRotation()));
    // 5. Set Shooter Speed
    shooter.setTargetSpeed(solution.flywheelRPM());

    // Set both hoods (ProfiledPID will handle the angle)
    shooter.setHoodAngle(solution.hoodAngleRad());
  }
}
