package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public enum ArmConfiguration {
    // TODO
    L1(new Rotation2d(), 0, new Rotation2d()),
    L2(new Rotation2d(), 0, new Rotation2d()),
    L3(new Rotation2d(), 0, new Rotation2d()),
    L4(new Rotation2d(), 0, new Rotation2d()),
    HUMAN_PLAYER(new Rotation2d(), 0, new Rotation2d()),
    CLIMB(new Rotation2d(), 0, new Rotation2d());

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
