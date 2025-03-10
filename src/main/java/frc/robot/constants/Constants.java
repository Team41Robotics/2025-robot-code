package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

	public static final class RobotConstants {
		public static final double ROBOT_LENGTH = Units.inchesToMeters(28);
		public static final double ROBOT_WIDTH = Units.inchesToMeters(27);

		public static final double SWERVE_MAXSPEED = 4.42;
		public static final double ANGULAR_MAX_SPEED = SWERVE_MAXSPEED / (Math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2);

		public static final double SPEED_MULT = 0.5;
		public static final double TURBO_SPEED_MULT = 0.2;

		public static final double ANGULAR_SPEED_MULT = 0.75;
		public static final double TURBO_ANGULAR_SPEED_MULT = 0.3;

		public static final double SWERVE_WHEEL_RAD = Units.inchesToMeters(2);
		public static final double L3_DRIVE_RATIO = 1 / 5.36; // input RPM * gearing = output RPM
		public static final double L3_TURN_RATIO = 1 / 18.75;
	}

	public static final class ArmConstants {
		// TODO: SET ARM CAN IDS

		public static final int SHOULDER_1 = 22;
		public static final int SHOULDER_2 = 21;
		public static final int SHOULDER_3 = 20;
		public static final int SHOULDER_4 = 19;

		public static final int TELESCOPE_1 = 23;
		public static final int TELESCOPE_2 = 24;

		public static final int WRIST = 0;

		public static final double MAX_EXTENSION = 1.09;
		public static final double MIN_EXTENSION = 0.05;

		public static final Rotation2d MIN_ROTATION = new Rotation2d(0.175); // SHOULDER
		public static final Rotation2d MAX_ROTATION = new Rotation2d(Math.PI / 2 - .3);
	}

	public static final double MODULE_DRIVE_KP = 0.05;
	public static final double MODULE_DRIVE_KF = 0.23;
	public static final double MODULE_TURN_KP = 3;

	public static final double DRIVER_TURN_KP = 1.;

	public static final double CAMERA_HEIGHT = Units.inchesToMeters(13.5);
	public static final double MAX_ARM_EXTEND = 1; // WIP
	public static final double TELESCOPE_PULLEY_RADIUS = Units.inchesToMeters(1.273); // WIP
	public static final double TELESCOPE_GEAR_RATIO = 5;
	public static final double SHOULDER_GEAR_RATIO = (54 / 18) * (54 / 18) * (72 / 9); // Full Gear ratio

	public static final PPHolonomicDriveController PATH_FOLLOWER_CONFIG = new PPHolonomicDriveController(
			new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
			new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
			);

	// public static RobotConfig ROBOT_CONFIG;

	// static {
	// 	try {
	// 		ROBOT_CONFIG = RobotConfig.fromGUISettings();
	// 	} catch (Exception e) {
	// 		e.printStackTrace();
	// 	}
	// }
}
