package frc.robot.subsystems.algae;

import static java.lang.Math.PI;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
	private AlgaeIOSparkMax io;
	private AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

	private PIDController m_PID;
	private Optional<Rotation2d> targetRotation;

	public AlgaeSubsystem() {
		io = new AlgaeIOSparkMax();
		targetRotation = Optional.of(new Rotation2d(95 / 180. * PI));
		m_PID = new PIDController(1, 0, 0); // todo
		m_PID.enableContinuousInput(0, Math.PI*2);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		if (!targetRotation.isEmpty()) {
			double out = m_PID.calculate(
					inputs.algaeRotation.getRadians(),
					targetRotation.get().getRadians());
			Logger.recordOutput("Algae/Output", out);
			io.setAlgaeVoltage(out);
		}
		Logger.processInputs("Algae", inputs);
		Logger.recordOutput("Algae/Target", targetRotation.get());
	}

	public void setAlgaeRotation(Rotation2d target) {
		this.targetRotation = Optional.of(target);
	}

	public Rotation2d clampTargetAngle(Rotation2d target) {
		target = new Rotation2d(MathUtil.clamp(target.getRadians(), 0, Math.PI / 2));
		return target;
	}

	public void runIntake(boolean spit) {
		io.setIntakeVoltage(spit ? 4.0 : -4.0);
	}
}
