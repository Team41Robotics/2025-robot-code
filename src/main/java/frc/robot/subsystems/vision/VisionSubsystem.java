package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.robot;
import frc.robot.constants.LimelightConfiguration;

public class VisionSubsystem extends SubsystemBase {
	private LimelightConfiguration config;
	private Pose2d robotToField = new Pose2d();
	private double mt1Timestamp = 0.0;
	private static final MedianFilter xFilter = new MedianFilter(20);
	private static final MedianFilter yFilter = new MedianFilter(20);
	private static final MedianFilter rFilter = new MedianFilter(20);

	private int counter;

	public void init(LimelightConfiguration _config) {
		config = _config;
		counter = 0;
		resetFilters();
	}

	@Override
	public void periodic() {
		LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.Name);
		Pose2d pose;
		if (mt1 != null) {
			if(mt1.tagCount > 0){
				pose = mt1.pose;
				boolean rejectUpdate = false;
				if(mt1.rawFiducials[0].ambiguity > 1){
					rejectUpdate = true;
					System.out.println("REJECTED UPDATE, IS TOO UNCERTAIN");
				}
				if(isOutlier(pose) && !robot.isDisabled()){
					rejectUpdate = true;
					System.out.println("REJECTED UPDATE, IS OUTLIER");
				}
				if(!rejectUpdate){
					this.robotToField = pose;
					this.mt1Timestamp = mt1.timestampSeconds;
					drive.addLimelightMeasurement(robotToField, mt1Timestamp);
				}
			}else{
				resetFilters();
			}
		}
		Logger.recordOutput("/Odom/limelight/limelight_pose/" + config.Name, this.robotToField);
		Logger.recordOutput("Odom/limelight/limelight_y_median", yFilter.calculate(this.robotToField.getY()));
		Logger.recordOutput("Odom/limelight/limelight_x_median", xFilter.calculate(this.robotToField.getX()));
		Logger.recordOutput(
				"Odom/limelight/limelight_theta_median",
				rFilter.calculate(this.robotToField.getRotation().getRadians()));
	}

	public static Matrix<N3, N1> getMeasurementStdDev(double dist, double bearing) { // TODO
		return VecBuilder.fill(999999, 999999, 999999);
	}

	public boolean isOutlier(Pose2d pose){
		double x = pose.getX();
		double y = pose.getY();
		double r = pose.getRotation().getRadians();

		return (Math.abs(x - xFilter.calculate(x)) > 0.5 || Math.abs(y - yFilter.calculate(y)) > 0.5 || Math.abs(r - rFilter.calculate(r)) > 0.1);
	}

	public void resetFilters(){
		xFilter.reset();
		yFilter.reset();
		rFilter.reset();
	}

}