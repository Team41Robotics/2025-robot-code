package frc.robot.commands.drive;

import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.left_js;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Util;
import java.util.function.DoubleSupplier;

public class BackUp extends Command{
	
	private double startTime = 0;

	public BackUp(){
		addRequirements(drive);
    }
	

	/**
	 * Runs the drive command with the specified velocity components.
	 *
	 * @param vx The velocity component in the x-axis.
	 * @param vy The velocity component in the y-axis.
	 * @param w  The angular velocity component.
	 */
	public void run() {
		ChassisSpeeds speeds = new ChassisSpeeds(0.15,0,0);
		drive.drive(speeds);
	}

	@Override
	public void execute() {
		if(startTime == 0){
			startTime = System.currentTimeMillis();
		}
		run();
	}

	@Override
	public boolean isFinished(){
		return (System.currentTimeMillis() - this.startTime > 1000);
	}

	@Override
	public void end(boolean interrupted){
		startTime = 0;
		drive.drive(new ChassisSpeeds());
	}
}
