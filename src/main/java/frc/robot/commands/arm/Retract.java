package frc.robot.commands.arm;

import static frc.robot.RobotContainer.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.ArmConfiguration;

public class Retract extends SequentialCommandGroup {
	public Retract(ArmConfiguration _config) {

		super(
				new InstantCommand(() -> arm.setWristTargetRotation(arm.getWristAngle() + Math.PI / 2)),
				new WaitCommand(0.25),
				new InstantCommand(() -> arm.setWristTargetRotation(_config.WRIST_ROTATION)),
				new InstantCommand(() -> arm.setTargetExtension(_config.EXTENSION)),
				new WaitCommand(0.75),
				new InstantCommand(() -> arm.setShoulderTargetRotation(_config.SHOULDER_ROTATION)));
		addRequirements(arm);
	}
}
