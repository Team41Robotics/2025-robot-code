package frc.robot.subsystems.vision;

import static frc.robot.RobotContainer.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.constants.LimelightConfiguration;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
	private LimelightIO io;
	private LimelightConfiguration config;
	private final LimelightInputsAutoLogged inputs = new LimelightInputsAutoLogged();
	private Pose2d robotToField = new Pose2d();
	private double mt1Timestamp = 0.0;
	private boolean doRejectUpdate = false;

	public void init(LimelightConfiguration _config) {
		config = _config;
		io = new LimelightIO(config.Name);
		System.out.println("Initialized limelight with name, " + config.Name);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs(config.Name, inputs);

		// As it turns out, mt1 works a lot better than mt2 for u

		LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(config.Name);

		if (mt1.tagCount >= 1 && mt1.rawFiducials.length == 1) {
			if (mt1.rawFiducials[0].ambiguity > 0.7) {
				doRejectUpdate = true;
				// System.out.println("Rejected update: Target too ambiguous");
			}
			if (mt1.rawFiducials[0].distToCamera > 3) { // TODO
				doRejectUpdate = true;
				// System.out.println("Rejected update: Target too far");
			}
		}
		if (!hasTarget()) {
			doRejectUpdate = true;
			// System.out.println("Rejected update: No targets");
		}
		if (!doRejectUpdate) {
			robotToField = mt1.pose;
			mt1Timestamp = mt1.timestampSeconds;
			drive.addLimelightMeasurement(robotToField, mt1Timestamp);
			// System.out.println("Sent measurement");
		}
		Logger.recordOutput("/Odom/limelight_pose/" + config.Name, this.robotToField);
	}

	public boolean hasTarget() {
		return inputs.hasTarget;
	}

	public double getYawRadians() {
		return inputs.yaw;
	}

	public double getPitchRadians() {
		return inputs.pitch;
	}

	public Optional<Pose2d> getEstimatePose() {
		return Optional.of(this.robotToField);
	}

	public int getTargetID() {
		return inputs.iD;
	}
}
