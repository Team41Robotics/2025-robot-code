package frc.robot.subsystems.algae;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
	private AlgaeIOSparkMax io;
	private AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

	private PIDController m_PID;
	private Optional<Rotation2d> targetRotation;

	public AlgaeSubsystem() {
		io = new AlgaeIOSparkMax();
		targetRotation = Optional.of(new Rotation2d(Math.PI/4));
		m_PID = new PIDController(2, 0, 1); // todo
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		if (!targetRotation.isEmpty()) {
			double out = m_PID.calculate(
					inputs.algaeRotation.getRadians(),
					clampTargetAngle(targetRotation.get()).getRadians());
			io.setAlgaeVoltage(out);
		}
        updateLogging();
	}

    public void updateLogging(){
        Logger.recordOutput("/Algae/Rotation", inputs.algaeRotation.getRadians());
	Logger.recordOutput("/Algae/PivotVoltage", inputs.algaePivotVoltage);
	Logger.recordOutput("/Algae/Pivot Target", this.targetRotation.get().getRadians());
    }

    public void setAlgaeRotation(Rotation2d target){
        this.targetRotation = Optional.of(target);
    }

	public Rotation2d clampTargetAngle(Rotation2d target) {
		target = new Rotation2d(MathUtil.clamp(target.getRadians(), 0, Math.PI/2));
		return target;
	}

    public void runIntake(boolean spit){
        io.setIntakeVoltage(spit ? 4.0 : -4.0);
    }

}
