package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.L4;
import static frc.robot.RobotContainer.intake;

public class ScoreCoral extends Command {

	public ScoreCoral() {
		addRequirements(intake);
	}

	@Override
	public void execute() {
		if (!intake.isBeamBreakTriggered()) {
			return;
		}
		if (L4) {
			intake.runMotor(-0.9);
		} else {
			intake.runMotor(0.9);
		}
	}

	@Override
	public boolean isFinished() {
		if (intake.isBeamBreakTriggered()) {
			intake.stopMotors();
			return true;
		}
		return false;
	}
}
