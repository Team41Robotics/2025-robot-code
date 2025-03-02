package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

	private IntakeIOSparkMax io;
	private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

	public IntakeSubsystem() {
		io = new IntakeIOSparkMax();

	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		updateLogging();
	}

	public void updateLogging(){
		Logger.recordOutput("/Intake/Beam Break Status", inputs.beamBreakIsDisabled);
	}

	public void runMotor(double velocity, boolean hawktuah) {
		io.setVelocity(hawktuah ? velocity : -velocity);
	}

	public boolean getBeamBreak(){
		return inputs.beamBreakIsDisabled;
	}

	

}
