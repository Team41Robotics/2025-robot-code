package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.RobotContainer.drive;
import frc.robot.constants.LimelightConfiguration;

public class VisionSubsystem extends SubsystemBase {
	private LimelightConfiguration config;
	private Pose2d robotToField = new Pose2d();
	private double mt1Timestamp = 0.0;
	private static final MedianFilter xFilter = new MedianFilter(10);
	private static final MedianFilter yFilter = new MedianFilter(10);
	private static final MedianFilter rFilter = new MedianFilter(10);

	private final double timeThreshold = 5000; // milliseconds 

	private double timeSinceLastTag = System.currentTimeMillis(); // seconds
	private int measurementCounter;
	private int outlierCounter;
	private double xSum;
	private double ySum;
	private double xMean;
	private double yMean;


	public void init(LimelightConfiguration _config) {
		config = _config;
		outlierCounter = 0;
		measurementCounter = 0;
		xSum = 0;
		ySum = 0;
		resetFilters();
	}

	@Override
	public void periodic() {
		boolean rejectUpdate = false;
		LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.Name);
		Pose2d pose;
		if (mt1 != null) {
			if(mt1.tagCount > 0){
				measurementCounter ++;
				timeSinceLastTag = System.currentTimeMillis() - timeSinceLastTag;
				pose = mt1.pose;
				xSum += pose.getX();
				ySum += pose.getY();
				xFilter.calculate(pose.getX());
				yFilter.calculate(pose.getY());
				rFilter.calculate(pose.getRotation().getRadians());
				if(mt1.rawFiducials[0].ambiguity > 1){
					rejectUpdate = true;
					System.out.println("REJECTED UPDATE, IS TOO UNCERTAIN");
				}
				if(isOutlier(pose) && timeSinceLastTag < timeThreshold){
					rejectUpdate = true;
					System.out.println(config.Name + " REJECTED UPDATE, IS OUTLIER " + ++outlierCounter);
				}
				if(!rejectUpdate){
					double[] stddevs = NetworkTableInstance.getDefault().getTable(config.Name).getEntry("stddevs").getDoubleArray(new double[12]);
					double stdDevX = stddevs[0];
					double stdDevY = stddevs[1];
					double stdDevZ = stddevs[2];
					this.robotToField = pose;
					this.mt1Timestamp = mt1.timestampSeconds;		
					drive.addLimelightMeasurement(robotToField, mt1Timestamp, VecBuilder.fill(stdDevX, stdDevY, stdDevZ));
				}
			}else{
				timeSinceLastTag = System.currentTimeMillis();
				resetFilters();
			}
		}
		Logger.recordOutput("/Odom/limelight/limelight_pose/" + config.Name, this.robotToField);
		Logger.recordOutput("Odom/limelight/limelight_y_median", yFilter.lastValue());
		Logger.recordOutput("Odom/limelight/limelight_x_median", xFilter.lastValue());
		Logger.recordOutput(
				"Odom/limelight/limelight_theta_median",
				rFilter.calculate(this.robotToField.getRotation().getRadians()));
	
	}

	public boolean isOutlier(Pose2d pose){
		double x = pose.getX();
		double y = pose.getY();

		return (Math.abs(x - xFilter.lastValue()) > 0.25 || Math.abs(y - yFilter.lastValue()) > 0.25);
	}

	public void resetFilters(){
		xFilter.reset();
		yFilter.reset();
		rFilter.reset();
	}

}