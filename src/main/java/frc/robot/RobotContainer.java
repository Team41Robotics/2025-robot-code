package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.arm.Retract;
import frc.robot.commands.arm.ScoreCoral;
import frc.robot.commands.arm.SetToScore;
import frc.robot.commands.drive.AlignToStation;
import frc.robot.commands.drive.DefaultDrive;
import frc.robot.constants.ArmConfiguration;
import frc.robot.constants.LimelightConfiguration;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.LocalADStarAK;

public class RobotContainer {
	public static Robot robot;
	public static SwerveSubsystem drive = new SwerveSubsystem();
	public static IMU imu = new IMU();
	//public static AlgaeSubsystem algae = new AlgaeSubsystem();
	public static ArmSubsystem arm = new ArmSubsystem();
	public static IntakeSubsystem intake = new IntakeSubsystem();

	public static LimelightConfiguration config = new LimelightConfiguration();
	public static LimelightConfiguration config2 = new LimelightConfiguration();
	public static VisionSubsystem limelight1 = new VisionSubsystem();
	public static VisionSubsystem limelight2 = new VisionSubsystem();

	public static CommandJoystick left_js = new CommandJoystick(4);
	public static CommandJoystick right_js = new CommandJoystick(3);
	public static CommandJoystick ds = new CommandJoystick(2);

	public static boolean L4 = false;

	public static LoggedDashboardChooser<Command> autoChooser;
	public static LoggedDashboardChooser<Command> reefChooser;
 
	public static void initSubsystems() {

		config.		setName("limelight-front")
				.withHeightOffset(0.28)
				.withLengthOffset(0.3)
				.withWidthOffset(0.15);
		// arm.zero();
		config2.setName("limelight-back")
		.withHeightOffset(0.275)
		.withLengthOffset(-.025)
		.withWidthOffset(0.1)
		.withMountingYaw(Math.PI);


		drive.setDefaultCommand(new DefaultDrive(() -> left_js.getY(), () -> left_js.getX(), () -> -right_js.getX()));
		drive.init(new Pose2d());

		limelight1.init(config);
		limelight2.init(config2);

		Pathfinding.setPathfinder(new LocalADStarAK());
		autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());
		reefChooser = new LoggedDashboardChooser<>("Target on Reef", new SendableChooser<>());

		autoChooser.addOption("Goto Tag 13", new AlignToStation(13));
		autoChooser.addOption("Goto Tag 12", new AlignToStation(12));
		autoChooser.addOption("Goto Tag 2", new AlignToStation(2));
		autoChooser.addOption("Goto Tag 1", new AlignToStation(1));
		
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		reefChooser.addOption();
		
		


		configureBindings();
	}

	private static void configureBindings() {
		right_js.button(4).onTrue(new DeferredCommand(() -> autoChooser.get(), Set.of(drive)));
		// ds.button(12).onTrue(new InstantCommand(() -> arm.setTargetExtension(MAX_EXTENSION)));
		 ds.button(11).onTrue(new Retract(ArmConfiguration.HUMAN_PLAYER));			
		 ds.button(12).onTrue(new SetToScore(ArmConfiguration.L4));
		 ds.button(9).onTrue(new Retract(ArmConfiguration.L2));			
		 ds.button(10).onTrue(new SetToScore(ArmConfiguration.L3));
		 ds.button(8).onTrue(new Retract(ArmConfiguration.L1));			
		 ds.button(7).onTrue(new SetToScore(ArmConfiguration.NEUTRAL));
		 
		// ds.button(11).onTrue(new InstantCommand(() -> arm.setTargetExtension(MIN_EXTENSION)));
		// ds.button(12).onTrue(new InstantCommand(() -> arm.setTargetExtension(MAX_EXTENSION)));
		// ds.button(11).onTrue(new InstantCommand(() -> algae.setAlgaeRotation(new Rotation2d(1.47))));
		right_js.button(1).whileTrue(new InstantCommand(() -> intake.runMotor(0.10)).until(() -> !intake.isBeamBreakNotTriggered()).andThen(new InstantCommand(() -> intake.stopMotors()))); // INTAKE
		left_js.button(1).onTrue(new ScoreCoral());
	}

	public static Command getAutonomousCommand() {
		return autoChooser.get();
	}
}
