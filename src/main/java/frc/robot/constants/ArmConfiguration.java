package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;

public enum ArmConfiguration {
    // TODO
    NEUTRAL(new Rotation2d(Units.degreesToRadians(1)), MIN_EXTENSION, new Rotation2d()),
    L1(new Rotation2d(Units.degreesToRadians(1)), MIN_EXTENSION, new Rotation2d()),
    L2(new Rotation2d(Units.degreesToRadians(83)), MIN_EXTENSION, new Rotation2d()),
    L3(new Rotation2d(Units.degreesToRadians(75)),Units.inchesToMeters(18) , new Rotation2d()),
    L4(new Rotation2d(Units.degreesToRadians(67.5)), MAX_EXTENSION, new Rotation2d()),
    HUMAN_PLAYER(new Rotation2d(Units.degreesToRadians(37.5)), MIN_EXTENSION, new Rotation2d()),
    CLIMB(new Rotation2d(Units.degreesToRadians(68)), Units.inchesToMeters(22), new Rotation2d());

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
