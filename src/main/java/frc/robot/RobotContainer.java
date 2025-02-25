package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.drive.AlignToReef;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.constants.LimelightConfiguration;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LocalADStarAK;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive = new SwerveSubsystem();
	public static IMU imu = new IMU();
	public static ArmSubsystem arm = new ArmSubsystem();

	public static LimelightConfiguration config = new LimelightConfiguration();
	//public static LimelightConfiguration config2 = new LimelightConfiguration();
	public static VisionSubsystem limelight1 = new VisionSubsystem();
	//public static VisionSubsystem limelight2 = new VisionSubsystem();

	public static CommandJoystick left_js = new CommandJoystick(4);
	public static CommandJoystick right_js = new CommandJoystick(3);
	public static CommandJoystick ds = new CommandJoystick(2);

	public static LoggedDashboardChooser<Command> autoChooser;

	public static void initSubsystems() {

		config.withHeightOffset(Units.inchesToMeters(8.375))
				.withLengthOffset(Units.inchesToMeters(13.75))
				.withWidthOffset(Units.inchesToMeters(.25));
		//arm.zero();
		// config2.setName("limelight2")
		// .withHeightOffset(Units.inchesToMeters(8.375))
		// .withLengthOffset(Units.inchesToMeters(14))
		// .withWidthOffset(Units.inchesToMeters(4.75));

		drive.setDefaultCommand(new DefaultDrive(() -> left_js.getY(), () -> left_js.getX(), () -> -right_js.getX()));
		drive.init(new Pose2d());

		limelight1.init(config);
		// limelight2.init(config2);

		Pathfinding.setPathfinder(new LocalADStarAK());
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());

		autoChooser.addOption("Goto Tag 20", new AlignToReef(20));
		autoChooser.addOption("Goto Tag 21", new AlignToReef(21));
		autoChooser.addOption("Goto Tag 22", new AlignToReef(22));

		configureBindings();
	}

	private static void configureBindings() {
		right_js.button(4).onTrue(new DeferredCommand(() -> autoChooser.get(), Set.of(drive)));
		left_js.button(4).onTrue(new InstantCommand(() -> arm.setShoulderTargetRotation(new Rotation2d(Math.PI/4))));
		left_js.button(3).onTrue(new InstantCommand(() -> arm.setShoulderTargetRotation(new Rotation2d(0))));
		left_js.button(2).onTrue(new InstantCommand(() -> arm.setTargetExtension(0.3)));
		left_js.button(1).onTrue(new InstantCommand(() -> arm.setTargetExtension(0)));


	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
