package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;

public enum ArmConfiguration {
	// TODO
	NEUTRAL(MIN_ROTATION, MIN_EXTENSION, 4.455, "NEUTRAL"),
	L1(MIN_ROTATION, MIN_EXTENSION, 0, "L1"),
	L2(Rotation2d.fromRadians(1.615), MIN_EXTENSION, 4.3, "L2"),
	L3(Rotation2d.fromRadians(1.435), Units.inchesToMeters(18), 4.3, "L3"),
	L4(Rotation2d.fromRadians(1.2), MAX_EXTENSION, 1.2, "L4"),
	HUMAN_PLAYER(Rotation2d.fromRadians(.82404), MIN_EXTENSION, 3.63, "HP"),
	CLIMB(Rotation2d.fromDegrees(68), Units.inchesToMeters(22), 0, "CLIMB");
	public Rotation2d SHOULDER_ROTATION;
	public double WRIST_ROTATION;
	public double EXTENSION;
	public String NAME;

	/**
	 * Constructs a SwerveModuleConfiguration with the specified parameters.
	 *
	 * @param encoder     the encoder ID
	 * @param turnMotor   the turn motor ID
	 * @param drive_motor the drive motor ID
	 * @param offset_rot  the offset rotation angle in radians
	 */
	ArmConfiguration(Rotation2d _shoulder_rotation, double _extension, double _wrist_rotation, String _name) {
		SHOULDER_ROTATION = _shoulder_rotation;
		EXTENSION = _extension;
		WRIST_ROTATION = _wrist_rotation;
		NAME = _name;
	}
}
