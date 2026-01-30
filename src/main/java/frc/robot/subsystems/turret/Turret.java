package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

// import com.team6911.easycrt.EasyCRT;
// import com.team6911.easycrt.EasyCRTConfig;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  // Active Tracking Controller
  private final ProfiledPIDController controller;
  private boolean isZeroed = false;

  // Yams CRT Solver
  private final EasyCRT crtSolver;

  public Turret(TurretIO io) {
    this.io = io;

    var easyCrtConfig =
        new EasyCRTConfig(
                () -> Rotations.of(inputs.encoder1Rotations),
                () -> Rotations.of(inputs.encoder2Rotations))
            // Configure Gearing
            .withCommonDriveGear(
                1.0,
                TurretConstants.kTurretGearTeeth,
                TurretConstants.kEncoder1Teeth,
                TurretConstants.kEncoder2Teeth)
            // Define the valid range of motion
            .withMechanismRange(
                Rotations.of(Units.radiansToRotations(TurretConstants.kMinAngleRad)),
                Rotations.of(Units.radiansToRotations(TurretConstants.kMaxAngleRad)))
            // Set tolerance for the CRT match (adjust based on backlast)
            .withMatchTolerance(Rotations.of(.05))
            // Set Offsets here after mechanical zeroing (NOT on the device)
            .withAbsoluteEncoderOffsets(Rotations.of(0.0), Rotations.of(0.0));

    // Initialize the solver with the config
    this.crtSolver = new EasyCRT(easyCrtConfig);

    // Setup Profiled PID for Active Tracking
    controller =
        new ProfiledPIDController(
            5.0,
            0.0,
            0.0, // Tune kP, kI, kD (along with FF values)
            new TrapezoidProfile.Constraints(
                TurretConstants.kMaxVelocityRadPerSec, TurretConstants.kMaxAccelRadPerSec));

    // Enables Wrapping in the Turret (Make sure the robot can handle it physically before starting)
    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // Update and Log Inputs
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Attempt to Zero on Startup (should ONLY do once)
    if (!isZeroed && inputs.encodersConnected) {
      seedPositionFromCRT();
    }

    // Control Loop
    if (isZeroed) {
      double voltage = controller.calculate(inputs.positionRad);
      io.setVoltage(voltage);

      Logger.recordOutput("Turret/Setpoint", controller.getSetpoint().position);
    } else {
      io.setVoltage(0.0);
    }
  }

  // Uses YAMS EasyCRT to find absolute angle and seeds the sparkmax internal encoder
  private void seedPositionFromCRT() {
    // The Solver runs here using the suppliers defined in the config
    var angleOpt = crtSolver.getAngleOptional();

    angleOpt.ifPresent(
        mechAngle -> {
          // Convert YAMS Angle (Rotations) to Radians
          double angleRad = mechAngle.in(Rotations) * 2.0 * Math.PI;

          // Seed the Internal encoder
          io.setInternalPosition(angleRad);

          isZeroed = true;
          Logger.recordOutput("Turret/CRT_Seeded", true);
          Logger.recordOutput("Turret/CRT_Angle_Rad", angleRad);
        });

    if (angleOpt.isEmpty()) {
      Logger.recordOutput("Turret/CRT_Error", true);
    }
  }

  public void setTargetPosition(double angleRad) {
    if (isZeroed) {
      controller.setGoal(angleRad);
    }
  }
}
