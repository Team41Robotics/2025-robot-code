package frc.robot.commands.drive;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.drive;
import static frc.robot.util.Util.convertAngle;
import static frc.robot.util.Util.getAdjustedPoseHumanPlayer;
import static frc.robot.util.Util.getAprilTagPose;

@SuppressWarnings("FieldMayBeFinal")
public class AlignToStation extends Command {

	// TODO: Adjust PID gains

	private PIDController xPID = new PIDController(0.2, 0, 0);
	private PIDController yPID = new PIDController(0.2, 0, 0);
	private PIDController wPID = new PIDController(0.2, 0., 0.);

	private Optional<Pose2d> target_pose;
	private Optional<Pose2d> stored_pose = Optional.empty();
	private int target_id;

	private Pose2d adj_pose;

	public AlignToStation(int target_id) {
		addRequirements(drive);
		wPID.enableContinuousInput(0, 2 * Math.PI);
		this.target_id = target_id;
	}

	@Override
	public void initialize() {
		// target_pose = photon.getAprilTagPose();
		// target_pose = photon.getAprilTagPose(20);
		target_pose = getAprilTagPose(target_id);
		if (target_pose.isEmpty()
				&& stored_pose
						.isEmpty()) { // For reliability, if not receiving new pose from PhotonVision, use previously
			// saved pose if any as reference
			return;
		} else if (target_pose.isEmpty() && !stored_pose.isEmpty()) {
			// System.out.println(drive.getPose().getTranslation().getDistance(stored_pose.get().getTranslation()));
			target_pose = stored_pose;
		}

		adj_pose = getAdjustedPoseHumanPlayer(target_pose.get());
	}

	@Override
	public void execute() {

		Pose2d current_pose = drive.getPose();
		double curr_X = current_pose.getX();
		double curr_Y = current_pose.getY();

		double adj_X = adj_pose.getX();
		double adj_Y = adj_pose.getY();

		double curr_rot = current_pose.getRotation().getRadians();
		double target_rot = adj_pose.getRotation().getRadians();

		Logger.recordOutput("/Odom/adjusted pose", adj_pose);
		Logger.recordOutput("/Odom/adjusted_pose/x", adj_X);
		Logger.recordOutput("/Odom/adjusted_pose/y", adj_Y);
		Logger.recordOutput("/Odom/adjusted_pose/w", adj_pose.getRotation().getRadians());

		double xVel = xPID.calculate(curr_X, adj_X);
		double yVel = xPID.calculate(curr_Y, adj_Y);
		double wVel = xPID.calculate(curr_rot, target_rot);

		drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
				xVel, yVel, wVel, drive.getPose().getRotation()));
	}

	@Override
	public boolean isFinished() {

		if (target_pose.isEmpty()) {
			return true;
		}

		Pose2d current_pose = drive.getPose();
		double dX = Math.abs(current_pose.getX() - adj_pose.getX());
		double dY = Math.abs(current_pose.getY() - adj_pose.getY()); // Translational difference

		double angle_offset = Math.abs(convertAngle(current_pose.getRotation().getRadians())
				- convertAngle(adj_pose.getRotation().getRadians())); // Angular difference

		if (dX < 0.15 && dY < 0.1 && angle_offset <= Units.degreesToRadians(5)) {
			System.out.println("Aligned");
			return true;
		}
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		super.end(interrupted);
		drive.drive(new ChassisSpeeds());
		stored_pose = Optional.empty();
	}
}
