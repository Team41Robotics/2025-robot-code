package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

	private IntakeIOSparkMax io;
	private IntakeIOInputsAutoLogged inputs;

	public IntakeSubsystem() {
		io = new IntakeIOSparkMax();
		inputs = new IntakeIOInputsAutoLogged();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		if (inputs.beamBreakIsActive == true) {
			io.stopMotor();
		}
	}
}
