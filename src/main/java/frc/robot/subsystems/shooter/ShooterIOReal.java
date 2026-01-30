package frc.robot.subsystems.shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOReal implements ShooterIO {
  // Primary Set
  private final SparkMax primaryLeader;
  private final SparkMax primaryFollower;

  // Secondary Set (Optional)
  private final SparkMax secondaryLeader;
  private final SparkMax secondaryFollower;

  // Hoods
  private final SparkMax primaryHood;
  private final SparkMax secondaryHood;

  public ShooterIOReal() {
    // --- 1. Primary Flywheel Setup ---
    primaryLeader = new SparkMax(ShooterConstants.kPrimaryLeaderID, MotorType.kBrushless);
    primaryFollower = new SparkMax(ShooterConstants.kPrimaryFollowerID, MotorType.kBrushless);

    // Configure Leader
    configureLeader(primaryLeader);

    // Configure Follower (Follows Leader, Inverted = true usually for shooters on opposite sides)
    configureFollower(primaryFollower, primaryLeader, true);

    // --- 2. Secondary Flywheel Setup ---
    if (ShooterConstants.kIsDoubleFlywheel) {
      secondaryLeader = new SparkMax(ShooterConstants.kSecondaryLeaderID, MotorType.kBrushless);
      secondaryFollower = new SparkMax(ShooterConstants.kSecondaryFollowerID, MotorType.kBrushless);

      configureLeader(secondaryLeader);
      configureFollower(secondaryFollower, secondaryLeader, true);
    } else {
      secondaryLeader = null;
      secondaryFollower = null;
    }

    // --- 3. Hood Setup ---
    if (ShooterConstants.kHasHood) {
      primaryHood = new SparkMax(ShooterConstants.kPrimaryHoodID, MotorType.kBrushless);
      configureHood(primaryHood);
    } else {
      primaryHood = null;
    }

    if (ShooterConstants.kHasHood && ShooterConstants.kHasDualHoods) {
      secondaryHood = new SparkMax(ShooterConstants.kSecondaryHoodID, MotorType.kBrushless);
      configureHood(secondaryHood);
    } else {
      secondaryHood = null;
    }
  }

  private void configureLeader(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast);

    // High current limit for Bang-Bang Recovery
    config.smartCurrentLimit(60);
    // Apply Config
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void configureFollower(SparkMax follower, SparkMax leader, boolean inverted) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(60);

    config.follow(leader, inverted);
    follower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Configure Hood (Brake, PID conversions)
  private void configureHood(SparkMax motor) {
    SparkMaxConfig config = new SparkMaxConfig();

    // Hoods must Brake to hold angle
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(20);

    // Convert Motor Rotations -> Hood Radians
    double posFactor = (1.0 / ShooterConstants.kHoodGearRatio) * 2.0 * Math.PI;
    config.encoder.positionConversionFactor(posFactor);
    config.encoder.velocityConversionFactor(posFactor / 60);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // -- LOGIC --

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Primary Flywheel
    inputs.primaryLeaderRPM = primaryLeader.getEncoder().getVelocity();
    inputs.primaryLeaderVolts = primaryLeader.getAppliedOutput() * primaryFollower.getBusVoltage();
    inputs.primaryLeaderAmps = new double[] {primaryFollower.getOutputCurrent()};

    inputs.primaryFollowerVolts =
        primaryFollower.getAppliedOutput() * primaryFollower.getBusVoltage();
    inputs.primaryFollowerAmps = new double[] {primaryFollower.getOutputCurrent()};

    // Secondary Flywheel
    if (secondaryLeader != null) {
      inputs.secondaryLeaderRPM = secondaryLeader.getEncoder().getVelocity();
      inputs.secondaryLeaderVolts =
          secondaryLeader.getAppliedOutput() * secondaryLeader.getBusVoltage();
      inputs.secondaryLeaderAmps = new double[] {secondaryFollower.getOutputCurrent()};

      inputs.secondaryFollowerVolts =
          secondaryFollower.getAppliedOutput() * secondaryFollower.getBusVoltage();
      inputs.secondaryFollowerAmps = new double[] {secondaryFollower.getOutputCurrent()};
    }

    // Primary Hood
    if (primaryHood != null) {
      inputs.primaryHoodRad = primaryHood.getEncoder().getPosition();
      inputs.primaryHoodVolts = primaryHood.getAppliedOutput() * primaryHood.getBusVoltage();
      inputs.hoodsConnected = true;
    }

    // Secondary Hood
    if (secondaryHood != null) {
      inputs.secondaryHoodRad = secondaryHood.getEncoder().getPosition();
      inputs.secondaryHoodVolts = secondaryHood.getAppliedOutput() * secondaryHood.getBusVoltage();
    }
  }

  @Override
  public void setPrimaryVolts(double volts) {
    primaryLeader.setVoltage(volts);
  }

  @Override
  public void setSecondaryVolts(double volts) {
    if (secondaryLeader != null) secondaryLeader.setVoltage(volts);
  }

  @Override
  public void setPrimaryHoodVolts(double volts) {
    if (primaryHood != null) primaryHood.setVoltage(volts);
  }

  @Override
  public void setSecondaryHoodVolts(double volts) {
    if (secondaryHood != null) secondaryHood.setVoltage(volts);
  }
}
