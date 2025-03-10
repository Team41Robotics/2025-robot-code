package frc.robot.subsystems.vision;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.LimelightConfiguration;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
	private LimelightConfiguration config;
	private Pose2d robotToField = new Pose2d();
	private double mt1Timestamp = 0.0;
	private final MedianFilter xFilter = new MedianFilter(30);
	private final MedianFilter YFilter = new MedianFilter(30);
	private final MedianFilter rFilter = new MedianFilter(30);

	public void init(LimelightConfiguration _config) {
		config = _config;
	}

	@Override
	public void periodic() {
		LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.Name);

		boolean doRejectUpdate = false;
		if (mt1 != null) {
			Pose2d pose = mt1.pose;
			if (mt1.tagCount == 1) {
				if (mt1.rawFiducials[0].ambiguity > .50) {
					doRejectUpdate = true;
				}
			}
			if (mt1.tagCount == 0) {
				doRejectUpdate = true;
			}

			if (!robot.isDisabled()
					&& (Math.abs(pose.getY() - YFilter.calculate(pose.getY())) > 0.5
							|| Math.abs(pose.getX() - xFilter.calculate(pose.getX())) > 0.5)) {
				doRejectUpdate = true;
			}

			if (!doRejectUpdate) {
				robotToField = mt1.pose;
				mt1Timestamp = mt1.timestampSeconds;
				drive.addLimelightMeasurement(robotToField, mt1Timestamp);
			}
		}
		Logger.recordOutput("/Odom/limelight/limelight_pose/" + config.Name, this.robotToField);
		Logger.recordOutput("Odom/limelight/limelight_y_median", YFilter.calculate(this.robotToField.getY()));
		Logger.recordOutput("Odom/limelight/limelight_x_median", xFilter.calculate(this.robotToField.getX()));
		Logger.recordOutput(
				"Odom/limelight/limelight_theta_median",
				rFilter.calculate(this.robotToField.getRotation().getRadians()));
	}

	public static Matrix<N3, N1> getMeasurementStdDev(double dist, double bearing) { // TODO
		return VecBuilder.fill(999999, 999999, 999999);
	}
}
