package frc.robot.subsystems.turret;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class TurretIOReal implements TurretIO {
  private final SparkMax motor;
  private final DutyCycleEncoder enc1; // 18T
  private final DutyCycleEncoder enc2; // 17T
  private final TurretConstants constants;

  public TurretIOReal() {
    constants = new TurretConstants();
    motor = new SparkMax(constants.kMotorID, MotorType.kBrushless);
    enc1 = new DutyCycleEncoder(constants.kEncoder1);
    enc2 = new DutyCycleEncoder(constants.kEncoder2);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(40);

    // Convert the Motor Rotations to Radians
    double positionFactor = (1.0 / constants.kMotorGearRatio) * 2.0 * Math.PI;
    double velocityFactor = positionFactor / 60.0;

    config.encoder.positionConversionFactor(positionFactor);
    config.encoder.velocityConversionFactor(velocityFactor);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.positionRad = motor.getEncoder().getPosition();
    inputs.velocityRadPerSec = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = new double[] {motor.getOutputCurrent()};

    // Read the absolute encoders
    inputs.encoder1Rotations = enc1.get();
    inputs.encoder2Rotations = enc2.get();
    inputs.encodersConnected = enc1.isConnected() && enc2.isConnected();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void setInternalPosition(double positionRad) {
    motor.getEncoder().setPosition(positionRad);
  }
}
