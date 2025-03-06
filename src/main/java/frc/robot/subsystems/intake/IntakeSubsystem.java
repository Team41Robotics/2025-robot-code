package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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

	public void updateLogging() {
		Logger.processInputs("/Intake", inputs);
	}

	public void runMotor(double velocity) {
		io.setVelocity(velocity);
	}

	public void stopMotors() {
		io.setVelocity(0);
	}

	public Command runIntake(double speed) {
		return new StartEndCommand(() -> this.runMotor(speed), this::stopMotors);
	}

	public boolean isBeamBreakNotTriggered() {
		return inputs.beamBreakStatus;
	}
}
