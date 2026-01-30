// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // Flywheel Controllers (These use Bang-Bang bc it's the quickest)
  // Bang-Bang Controller: If Under speed, Full Power. If over power, 0 power
  private final BangBangController primaryBang;
  private final BangBangController secondaryBang;

  // Setpoints
  private double targetPrimaryRPM = 0.0;
  private double targetSecondaryPRM = 0.0;

  // Hood controllers (Profiled PID)
  private final ProfiledPIDController primaryHoodController;
  private final ProfiledPIDController secondaryHoodController;

  // Setpoints
  private double targetPrimaryHoodAngle = ShooterConstants.kHoodMinAngleRad;
  private double targetSecondaryHoodAngle = ShooterConstants.kHoodMinAngleRad;

  // Tunable Numbers
  // Hood PID
  private final LoggedTunableNumber hoodKp =
      new LoggedTunableNumber("Tuning/Shooter/Hood/kP", ShooterConstants.kHoodkP);
  private final LoggedTunableNumber hoodKi =
      new LoggedTunableNumber("Tuning/Shooter/Hood/kI", ShooterConstants.kHoodkI);
  private final LoggedTunableNumber hoodKd =
      new LoggedTunableNumber("Tuning/Shooter/Hood/kD", ShooterConstants.kHoodkD);

  // Flywheel Power (Bang-Bang Voltage)
  private final LoggedTunableNumber flyVolts =
      new LoggedTunableNumber("Tuning/Shooter/FlywheelVolts", ShooterConstants.kBangBangVoltage);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Initialize Bang-Bang Controllers
    primaryBang = new BangBangController(ShooterConstants.kFlywheelToleranceRPM);
    secondaryBang = new BangBangController(ShooterConstants.kFlywheelToleranceRPM);

    // Initialize Hood Controllers
    primaryHoodController = createHoodController();
    secondaryHoodController = createHoodController();
  }

  private ProfiledPIDController createHoodController() {
    return new ProfiledPIDController(
        ShooterConstants.kHoodkP,
        0.0,
        0.0,
        new TrapezoidProfile.Constraints(
            ShooterConstants.kHoodMaxVel, ShooterConstants.kHoodMaxAccel));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Refresh Inputs
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    // Update Tunables (This is for Live Tuning should make tuning faster)
    // Check if PID values changed on the dashboard
    if (hoodKp.hasChanged() || hoodKi.hasChanged() || hoodKd.hasChanged()) {
      primaryHoodController.setPID(hoodKp.get(), hoodKi.get(), hoodKd.get());
      secondaryHoodController.setPID(hoodKp.get(), hoodKi.get(), hoodKd.get());
    }

    // Run Flywheel Logic
    runFlywheel(primaryBang, inputs.primaryLeaderRPM, targetPrimaryRPM, true, flyVolts.get());

    if (ShooterConstants.kIsDoubleFlywheel) {
      runFlywheel(
          secondaryBang, inputs.secondaryLeaderRPM, targetSecondaryPRM, false, flyVolts.get());
    }

    // Run Hood Logic
    if (ShooterConstants.kHasHood) {
      runHood(primaryHoodController, inputs.primaryHoodRad, targetPrimaryHoodAngle, true);
    }

    if (ShooterConstants.kHasHood && ShooterConstants.kHasDualHoods) {
      runHood(secondaryHoodController, inputs.secondaryHoodRad, targetSecondaryHoodAngle, false);
    }

    // Log Goals
    Logger.recordOutput("Shooter/Goal/PrimaryRPM", targetPrimaryRPM);
    Logger.recordOutput("Shooter/Goal/SecondaryRPM", targetSecondaryPRM);
    Logger.recordOutput("Shooter/Goal/PrimaryHood", targetPrimaryHoodAngle);
  }

  // Core Bang-Bang Logic
  private void runFlywheel(
      BangBangController controller,
      double currentRPM,
      double target,
      boolean isPrimary,
      double voltageToUse) {
    // Only run if we actualy have a target (aooids jitter at 0 RPM)
    if (target > 50.0) {
      // .calculate() returns 1.0 if we need speed, 0.0 if we don't
      double demand = controller.calculate(currentRPM, target);

      // Multiply by Max Voltage (12V)
      double volts = demand * voltageToUse;

      if (isPrimary) io.setPrimaryVolts(volts);
      else io.setSecondaryVolts(volts);
    } else {
      // Idle Mode
      if (isPrimary) io.setPrimaryVolts(0.0);
      else io.setSecondaryVolts(0.0);
    }
  }

  // Core Hood Logic
  private void runHood(
      ProfiledPIDController controller, double currentRad, double targetrad, boolean isPrimary) {
    // Safety Clamp: This ensures we never command past physical limits
    double clampedGoal =
        Math.max(
            ShooterConstants.kHoodMinAngleRad,
            Math.min(ShooterConstants.kHoodMaxAngleRad, targetrad));

    double volts = controller.calculate(currentRad, clampedGoal);

    if (isPrimary) io.setPrimaryHoodVolts(volts);
    else io.setSecondaryHoodVolts(volts);

    // Log Setpoints
    String prefix = isPrimary ? "Shooter/PrimaryHood" : "Shooter/SecondaryHood";
    Logger.recordOutput(prefix + "/Setpoint", controller.getSetpoint().position);
  }

  // --- Commands ---

  // Set the main target speed
  // If double Flywheel is enabled, this sets BOTH to the same speed
  public void setTargetSpeed(double rpm) {
    this.targetPrimaryRPM = rpm;
    if (ShooterConstants.kIsDoubleFlywheel) {
      this.targetSecondaryPRM = rpm;
    }
  }

  // Sets different speeds for Top and Bottom Rollers
  // Thisi s useful  for controlling spin (backspin/topspin)
  public void setSplitSpeeds(double primaryRPM, double secondaryRPM) {
    this.targetPrimaryRPM = primaryRPM;
    this.targetSecondaryPRM = secondaryRPM;
  }

  // Sets the hood angle
  // If Dual hoods are enabled, this sets BOTH to the same angle
  public void setHoodAngle(double radians) {
    this.targetPrimaryHoodAngle = radians;
    if (ShooterConstants.kHasDualHoods) {
      this.targetSecondaryHoodAngle = radians;
    }
  }

  /**
   * @return True if flywheels are within tolerance of target
   */
  public boolean isAtSpeed() {
    boolean primaryReady =
        Math.abs(inputs.primaryLeaderRPM - targetPrimaryRPM)
            < ShooterConstants.kFlywheelToleranceRPM;

    if (ShooterConstants.kIsDoubleFlywheel) {
      boolean secondaryReady =
          Math.abs(inputs.secondaryLeaderRPM = targetSecondaryPRM)
              < ShooterConstants.kFlywheelToleranceRPM;
      return primaryReady && secondaryReady;
    }

    return primaryReady;
  }
}
