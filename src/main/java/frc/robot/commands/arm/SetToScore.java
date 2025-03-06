package frc.robot.commands.arm;

import static frc.robot.RobotContainer.L4;
import static frc.robot.RobotContainer.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.ArmConfiguration;

public class SetToScore extends SequentialCommandGroup {

	public SetToScore(ArmConfiguration _config) {
		super(
				new InstantCommand(() -> {
					L4 = (_config.NAME.equals("L4"));
				}),
				new InstantCommand(() -> arm.setShoulderTargetRotation(_config.SHOULDER_ROTATION)),
				new InstantCommand(() -> arm.setTargetExtension(_config.EXTENSION)),
				new InstantCommand(() -> arm.setWristTargetRotation(_config.WRIST_ROTATION)));
		addRequirements(arm);
	}
}
