package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;

public enum ArmConfiguration {
	// TODO
	NEUTRAL(MIN_ROTATION, MIN_EXTENSION, Rotation2d.fromRadians(4.555)),
	L1(MIN_ROTATION, MIN_EXTENSION, Rotation2d.fromRadians(0)),
	L2(Rotation2d.fromDegrees(83), MIN_EXTENSION, Rotation2d.fromRadians(4.4)),
	L3(Rotation2d.fromDegrees(75), Units.inchesToMeters(18), Rotation2d.fromRadians(4.4)),
	L4(Rotation2d.fromDegrees(67.5), MAX_EXTENSION, Rotation2d.fromRadians((1.4))),
	HUMAN_PLAYER(Rotation2d.fromDegrees(50), MIN_EXTENSION, Rotation2d.fromRadians(3.9)),
	CLIMB(Rotation2d.fromDegrees(68), Units.inchesToMeters(22), new Rotation2d());
	public Rotation2d SHOULDER_ROTATION, WRIST_ROTATION;
	public double EXTENSION;


	/**
	 * Constructs a SwerveModuleConfiguration with the specified parameters.
	 *
	 * @param encoder     the encoder ID
	 * @param turnMotor   the turn motor ID
	 * @param drive_motor the drive motor ID
	 * @param offset_rot  the offset rotation angle in radians
	 */
	ArmConfiguration(Rotation2d _shoulder_rotation, double _extension, Rotation2d _wrist_rotation) {
		SHOULDER_ROTATION = _shoulder_rotation;
		EXTENSION = _extension;
		WRIST_ROTATION = _wrist_rotation;
	}
}
