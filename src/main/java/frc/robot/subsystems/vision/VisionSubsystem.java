package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import static frc.robot.RobotContainer.drive;
import frc.robot.constants.LimelightConfiguration;

public class VisionSubsystem extends SubsystemBase {
	private LimelightConfiguration config;
	private Pose2d robotToField = new Pose2d();
	private double mt1Timestamp = 0.0;
	private boolean doRejectUpdate = false;

	public void init(LimelightConfiguration _config) {
		config = _config;
		System.out.println("Initialized limelight with name, " + config.Name);
	}

	@Override
	public void periodic() {
		LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.Name);

		if (mt1.tagCount >= 1 && mt1.rawFiducials.length == 1) {
			if (mt1.rawFiducials[0].ambiguity > 80.) {
				doRejectUpdate = true;
				System.out.println("Rejected update: Target too ambiguous");
			}
			if (mt1.rawFiducials[0].distToCamera > 3) { // TODO
				doRejectUpdate = true;
				System.out.println("Rejected update: Target too far");
			}
		}
		if (mt1.tagCount == 0) {
			doRejectUpdate = true;
			//System.out.println(config.Name + " Rejected update: No targets");
		}
		if (!doRejectUpdate) {
			robotToField = mt1.pose;
			mt1Timestamp = mt1.timestampSeconds;
			drive.addLimelightMeasurement(robotToField, mt1Timestamp);
			System.out.println("Sent measurement");
		}
		Logger.recordOutput("/Odom/limelight_pose/" + config.Name, this.robotToField);
	}

	public Optional<Pose2d> getEstimatePose() {
		return Optional.of(this.robotToField);
	}
}
