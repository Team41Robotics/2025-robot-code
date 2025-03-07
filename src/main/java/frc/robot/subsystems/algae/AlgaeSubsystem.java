package frc.robot.subsystems.algae;

import static java.lang.Math.PI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
	private AlgaeIOSparkMax io;
	private AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

	private PIDController m_PID;
	double x = 80; // TODO NEEDS TUNING
	private Optional<Rotation2d> targetRotation;

	public AlgaeSubsystem() {
		io = new AlgaeIOSparkMax();
		targetRotation = Optional.of(new Rotation2d(95 / 180. * PI));
		m_PID = new PIDController(2, 0, 0); // todo
		m_PID.enableContinuousInput(0, Math.PI * 2);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		if (!targetRotation.isEmpty()) {
			double out = m_PID.calculate(
					inputs.algaeRotation.getRadians(), targetRotation.get().getRadians());
			Logger.recordOutput("Algae/Output", out);
			io.setAlgaeVoltage(out);
		}
		if(hasAlgae()){
			stopMotors();
		}
		Logger.processInputs("Algae", inputs);
		Logger.recordOutput("Algae/Target", targetRotation.get());
		Logger.recordOutput("Algae/Current Rotation", inputs.algaeRotation.getRadians());

	}

	public void setAlgaeRotation(Rotation2d target) {
		this.targetRotation = Optional.of(target);
	}

	public Rotation2d clampTargetAngle(Rotation2d target) {
		target = new Rotation2d(MathUtil.clamp(target.getRadians(), 0, Math.PI / 2));
		return target;
	}

	public void runIntake(boolean spit) {
		io.setIntakeVoltage(spit ? 5.0 : -5.0);
		
	}

	public boolean hasAlgae(){
		if(Math.abs(inputs.intakeCurrent[0]) > x) return true;
		return false;
	}

	public void stopMotors(){
		io.setIntakeVelocity(0);
	}
}
