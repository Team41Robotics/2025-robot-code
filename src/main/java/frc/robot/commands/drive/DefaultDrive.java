package frc.robot.commands.drive;

import static frc.robot.RobotContainer.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.util.DriveFeedforwards;

public class DefaultDrive extends Command {
	DoubleSupplier vx_sup, vy_sup, w_sup;

	/**
	 * Constructs a new defaultDrive command that controls the robot in a field oriented style
	 * @param vx_sup a supplier for the vx component of velocity
	 * @param vy_sup a supplier for the vy component of velocity
	 * @param w_sup  a supplier for the desired rotation velocity
	 *
	 */
	public DefaultDrive(DoubleSupplier vx_sup, DoubleSupplier vy_sup, DoubleSupplier w_sup) {
		addRequirements(drive);
		this.vx_sup = vx_sup;
		this.vy_sup = vy_sup;
		this.w_sup = w_sup;
	}

	/**
	 * Runs the drive command with the specified velocity components.
	 *
	 * @param vx The velocity component in the x-axis.
	 * @param vy The velocity component in the y-axis.
	 * @param w  The angular velocity component.
	 */
	public void run(double vx, double vy, double w) {
		ChassisSpeeds speeds = Util.joystickToSpeeds(
				vx, vy, w, left_js.button(3).getAsBoolean(), drive.getPose().getRotation());
		drive.drive(speeds);
	}

	@Override
	public void execute() {
		run(vx_sup.getAsDouble(), vy_sup.getAsDouble(), w_sup.getAsDouble());
	}
}