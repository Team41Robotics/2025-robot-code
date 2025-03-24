package frc.robot.subsystems.vision;

import static frc.robot.RobotContainer.drive;

import java.util.concurrent.RejectedExecutionHandler;

import edu.wpi.first.math.Matrix;
import static frc.robot.RobotContainer.useVision;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.LimelightConfiguration;
import frc.robot.util.MovingAverage;
import frc.robot.util.StandardDeviationTool;

import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
	private LimelightConfiguration config;
	private Pose2d robotToField = new Pose2d();
	private double mt1Timestamp = 0.0;
	private static final MedianFilter xFilter = new MedianFilter(10);
	private static final MedianFilter yFilter = new MedianFilter(10);
	private static final MedianFilter rFilter = new MedianFilter(10);

	private final StandardDeviationTool xMeasuredStdDev = new StandardDeviationTool(10);
	private final StandardDeviationTool yMeasuredStdDev = new StandardDeviationTool(10);
	private final StandardDeviationTool zMeasuredStdDev = new StandardDeviationTool(10);

	private final StandardDeviationTool rMeasuredStdDev = new StandardDeviationTool(10);


	private final double timeThreshold = 5000; // milliseconds

	private double timeSinceLastTag = Double.MAX_VALUE;
	private int outlierCounter;


	public void init(LimelightConfiguration _config) {
		config = _config;
		outlierCounter = 0;
		resetFilters();
	}

	@Override
	public void periodic() {
		boolean rejectUpdate = false;
		LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.Name);
		Pose2d pose;
		if (mt1 != null) {
			if (mt1.tagCount > 0) {
				timeSinceLastTag = 0;
				pose = mt1.pose;
				xFilter.calculate(pose.getX());
				yFilter.calculate(pose.getY());
				rFilter.calculate(pose.getRotation().getRadians());
			
				if (mt1.rawFiducials[0].ambiguity > 1) {
					rejectUpdate = true;
					System.out.println("REJECTED UPDATE, IS TOO UNCERTAIN");
				}
				if (isOutlier(pose) && timeSinceLastTag < timeThreshold) {
					rejectUpdate = true;
					System.out.println(config.Name + " REJECTED UPDATE, IS OUTLIER " + ++outlierCounter);
				}
				if (!rejectUpdate) {
					double[] stddevs = NetworkTableInstance.getDefault()
							.getTable(config.Name)
							.getEntry("stddevs")
							.getDoubleArray(new double[12]);
					double stdDevX = stddevs[0];
					double stdDevY = stddevs[1];
					double stdDevZ = stddevs[2];
					double stdDevYaw = stddevs[5];

					xMeasuredStdDev.addSample(stdDevX);
					yMeasuredStdDev.addSample(stdDevY);
					zMeasuredStdDev.addSample(stdDevZ);
					rMeasuredStdDev.addSample(stdDevYaw);
					this.robotToField = pose;
					this.mt1Timestamp = mt1.timestampSeconds;
					Matrix<N3, N1> stddevsOut;
					if (areStdsOk(stdDevX, stdDevY)) {
						stddevsOut = VecBuilder.fill(stdDevX, stdDevY, stdDevZ);
					} else {
						double xCoeff = Math.abs(0.25 - xMeasuredStdDev.getStdDeviation()) * 500;
						double yCoeff = Math.abs(0.25 - yMeasuredStdDev.getStdDeviation()) * 500;
						double zCoeff = Math.abs(0.25 - zMeasuredStdDev.getStdDeviation()) * 500;
						stddevsOut = VecBuilder.fill(stdDevX * xCoeff, stdDevY * yCoeff, stdDevZ * zCoeff);
					}
					Logger.recordOutput("Odom/limelight/std dev x raw " + config.Name, stdDevX);
					Logger.recordOutput("Odom/limelight/std dev y raw " + config.Name, stdDevY);
					Logger.recordOutput("Odom/limelight/std dev x out " + config.Name, stddevsOut.get(0, 0));
					Logger.recordOutput("Odom/limelight/std dev y out " + config.Name, stddevsOut.get(1, 0));
					Logger.recordOutput("Odom/limelight/ave stddev x" + config.Name, xMeasuredStdDev.getStdDeviation());
					Logger.recordOutput("Odom/limelight/ave stddev y" + config.Name, yMeasuredStdDev.getStdDeviation());
					drive.addLimelightMeasurement(robotToField, mt1Timestamp, stddevsOut);

				}
			} else {

				timeSinceLastTag += 20;
				resetFilters();
	
			}
		}
		Logger.recordOutput("/Odom/limelight/limelight_pose/" + config.Name, this.robotToField);
		Logger.recordOutput("Odom/limelight/limelight_y_median", yFilter.lastValue());
		Logger.recordOutput("Odom/limelight/limelight_x_median", xFilter.lastValue());
	}

	/**
	 * Returns whether or not to consider a measurement an outlier 
	 * @param pose
	 * @return boolean representing if measurement is an outlier
	 */
	public boolean isOutlier(Pose2d pose) {
		double x = pose.getX();
		double y = pose.getY();

		double threshold = 0.25; // Needs tuning
		return (Math.abs(x - xFilter.lastValue()) > threshold || Math.abs(y - yFilter.lastValue()) > threshold);
	}

	
	 
	/**
	 * Returns whether or not to trust measurement based on standard deviations
	 * @param stdx
	 * @param stdy
	 * @return boolean representing whether or not the measurement is trustworthy
	 */
	public boolean areStdsOk(double stdx, double stdy) {
		return (Math.abs(stdx - xMeasuredStdDev.getStdDeviation()) > 0.25
				|| Math.abs(stdy - yMeasuredStdDev.getStdDeviation()) > 0.25);
	}

	/**
	 * Reset median filters 
	 */

	public void resetFilters() {
		xFilter.reset();
		yFilter.reset();
		rFilter.reset();
	}
}
