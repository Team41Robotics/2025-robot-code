package frc.robot.util;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.exp;
import static java.lang.Math.signum;
import static java.lang.Math.sin;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import static frc.robot.constants.Constants.RobotConstants.ANGULAR_MAX_SPEED;
import static frc.robot.constants.Constants.RobotConstants.ANGULAR_SPEED_MULT;
import static frc.robot.constants.Constants.RobotConstants.ROBOT_WIDTH;
import static frc.robot.constants.Constants.RobotConstants.SPEED_MULT;
import static frc.robot.constants.Constants.RobotConstants.SWERVE_MAXSPEED;
import static frc.robot.constants.Constants.RobotConstants.TURBO_ANGULAR_SPEED_MULT;
import static frc.robot.constants.Constants.RobotConstants.TURBO_SPEED_MULT;

public class Util {
	// tuned value for sigmoid, higher values make the curve steeper, this is what thomas likes. Use desmos to preview
	// curves
	private static double a = 1;

	/**
	 * Applies a sensitivity curve to a given input value.
	 * If the absolute value of the input is less than the deadband value, the output is 0.
	 * Otherwise, the output is calculated using a sigmoid function and the sign of the input.
	 *
	 * @param w the input value
	 * @param d the deadband value
	 * @return the output value after applying the sensitivity curve
	 */
	public static double sensCurve(double w, double d) {
		if (abs(w) < d) return 0;
		double wn = (abs(w) - d) / (1 - d);

		// wn = sigmoid(wn) / sigmoid(1);
		wn = wn * wn;
		return wn * signum(w);
	}

	/**
	 * Calculates the sigmoid function of a given input.
	 * The sigmoid function maps the input to a value between -1 and 1.
	 *
	 * @param x the input value
	 * @return the sigmoid value of the input
	 */
	public static double sigmoid(double x) {
		return 2 / (1.0 + exp(-x * a)) - 1;
	}

	/**
	 * Represents the speeds of a robot chassis in field-relative coordinates.
	 * The speeds include the linear velocity components in the x and y directions,
	 * as well as the angular velocity component around the robot's center of rotation.
	 *
	 * Also, the sensitivity curve is basically a clamped linear function right now :/
	 *
	 * @see <a href="https://www.desmos.com/calculator/mbxig6izt9">Desmos Graph</a>
	 * @param vx The linear velocity component in the x direction.
	 * @param vy The linear velocity component in the y direction.
	 * @param w The angular velocity component around the robot's center of rotation.
	 * @param turbo A boolean indicating whether the robot is in turbo mode.
	 * @param rot The rotation of the robot chassis.
	 * @return The ChassisSpeeds object representing the speeds of the robot chassis.
	 */
	public static ChassisSpeeds joystickToSpeeds(double vx, double vy, double w, boolean turbo, Rotation2d rot) {
		double mag = Math.hypot(vx, vy);
		double mag_curved = MathUtil.clamp(Util.sensCurve(mag, 0.15) * 1.5, -1, 1);

		double theta = Math.atan2(vy, vx);
		double sign = isRed() ? -1.0 : 1.0;

		double speed_mult = turbo ? TURBO_SPEED_MULT : SPEED_MULT;
		double angular_mult = turbo ? TURBO_ANGULAR_SPEED_MULT : ANGULAR_SPEED_MULT;
		ChassisSpeeds ret = ChassisSpeeds.fromFieldRelativeSpeeds(
				cos(theta) * mag_curved * SWERVE_MAXSPEED * speed_mult * sign,
				sin(theta) * mag_curved * SWERVE_MAXSPEED * speed_mult * sign,
				MathUtil.applyDeadband(w, 0.1) * ANGULAR_MAX_SPEED * angular_mult,
				rot);
		// ChassisSpeeds ret = new ChassisSpeeds(
		// 	cos(theta) * mag_curved * SWERVE_MAXSPEED * speed_mult * sign,
		// 	sin(theta) * mag_curved * SWERVE_MAXSPEED * speed_mult * sign,
		// 	MathUtil.applyDeadband(w, 0.1) * ANGULAR_MAX_SPEED * angular_mult);
		return ret;
	}

	/**
	 * Returns whether the robot is on the red alliance.
	 *
	 * @return true if the robot is on the red alliance, false otherwise
	 */
	public static boolean isRed() {
		return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
	}

	public static double convertAngle(double angle) { // Converts an angle over pi radians into a negative angle
		if (angle < Math.PI) {
			return angle;
		} else {
			return angle - (2 * Math.PI);
		}
	}

	static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

	public static Optional<Pose2d> getAprilTagPose(int id) {
		return Optional.of(fieldLayout.getTagPose(id).get().toPose2d());
	}

	public static Pose2d getAdjustedPose(Pose2d target, boolean isRight) {

		double endEffectorOffset = isRight ? 0.325 : 0.06;

		Pose2d returnable = target;
		Transform2d currentToTarget =
				new Transform2d(new Translation2d((ROBOT_WIDTH / 2), endEffectorOffset), new Rotation2d(0));
		return returnable.transformBy(currentToTarget);
	}

	public static Pose2d getAdjustedPoseHumanPlayer(Pose2d target) {

		// Need to add stuff for spacing
		// double extension = 0.12; // TODO (In meters)
		double effectorOffset = -0.2;
		Pose2d returnable = target;
		Transform2d currentToTarget =
				new Transform2d(new Translation2d((ROBOT_WIDTH / 2) + 0.18, effectorOffset), new Rotation2d(Math.PI));
		return returnable.transformBy(currentToTarget);
	}

	public static double rampVoltage(double curr, double prev) {
		if (Math.abs(curr - prev) > 0.2) {
			return MathUtil.clamp(curr, prev - .2, prev + .2);
		} else {
			return curr;
		}
	}
}
