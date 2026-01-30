package frc.robot.commands.ShooterExamples;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.ShootingPhysics;
import java.util.function.Supplier;

public class AimTurretSOTF extends Command {
  private final Turret turret;
  private final Drive drive;
  private final Shooter shooter;
  private final Supplier<Translation2d> targetSupplier;

  public AimTurretSOTF(
      Turret turret, Drive drive, Shooter shooter, Supplier<Translation2d> target) {
    this.turret = turret;
    this.drive = drive;
    this.shooter = shooter;
    this.targetSupplier = target;
    addRequirements(turret, shooter); // Drive is NOT required (it keeps moving)
  }

  @Override
  public void execute() {
    // 1. Get Inputs
    Pose2d robotPose = drive.getPose();
    ChassisSpeeds fieldSpeeds = drive.getFieldRelativeSpeeds();
    // 2.  Calculate SOTF Solution
    var solution = ShootingPhysics.calculateShot(robotPose, fieldSpeeds, targetSupplier.get());

    // 3. Convert to Turret-Relative Angle
    // The solution gives a Field-Relative angle. We must subtract robot heading.
    Rotation2d turretSetpoint = solution.azimuth().minus(robotPose.getRotation());

    // 4. Set Outputs
    turret.setTargetPosition(turretSetpoint.getRadians());

    // Adjust Shooter RPM based on the "Variable Velocity Artifact"
    // If driving away, this speed will be higher than stationary.
    // shooter.setTargetSpeed(solution.effectiveSpeed()); // Make the shooter subsystem
  }
}
