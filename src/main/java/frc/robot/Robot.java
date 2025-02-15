// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.imu;
import static frc.robot.RobotContainer.initSubsystems;
import frc.robot.util.Util;

public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;

	@Override
	public void robotInit() {
		Util.getAprilTagPose(1); // Dont ask lmao, removing this will kill the robot
		initSubsystems();
		RobotContainer.robot = this;
		Logger.recordMetadata("ProjectName", "Robot2025");
		if (isReal()) {
			// Logger.addDataReceiver(new WPILOGWriter("/D/logs"));
			Logger.addDataReceiver(new NT4Publisher());
		} else {
			// setUseTiming(false);
			String logPath = LogFileUtil.findReplayLog();
			// Logger.setReplaySource(new WPILOGReader(logPath));

			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
		}
		Logger.start();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void autonomousInit() {
		drive.zero();
		m_autonomousCommand = RobotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void testInit() {
		imu.zeroYaw();
		drive.getOffsets();
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}
}
