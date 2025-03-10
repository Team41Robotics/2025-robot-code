package frc.robot.commands.arm;

import static frc.robot.RobotContainer.L4;
import static frc.robot.RobotContainer.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCoral extends Command {

	public ScoreCoral() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		if (intake.isBeamBreakNotTriggered()) {
			return;
		}
		if (L4) {
			intake.runMotor(-0.75);
		} else {
			intake.runMotor(0.75);
		}
	}

	@Override
	public boolean isFinished() {
		if (intake.isBeamBreakNotTriggered()) {
			intake.stopMotors();
			return true;
		}
		return false;
	}
}
