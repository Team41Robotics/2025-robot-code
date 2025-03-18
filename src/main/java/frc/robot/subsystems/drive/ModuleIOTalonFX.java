package frc.robot.subsystems.drive;

import static frc.robot.constants.Constants.MODULE_DRIVE_KF;
import static frc.robot.constants.Constants.MODULE_DRIVE_KP;
import static frc.robot.constants.Constants.MODULE_TURN_KP;
import static frc.robot.constants.Constants.RobotConstants.L3_DRIVE_RATIO;
import static frc.robot.constants.Constants.RobotConstants.L3_TURN_RATIO;
import static frc.robot.constants.Constants.RobotConstants.SWERVE_WHEEL_RAD;
import static java.lang.Math.PI;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.SwerveModuleConfiguration;

/**
 * Module IO implementation for TalonFX drive motor controller, TalonFX turn motor controller, and
 * CANcoder absolute encoder connected to the RIO.
 */
public class ModuleIOTalonFX implements ModuleIO {
	private static final double DRIVE_GEAR_RATIO = 1 / L3_DRIVE_RATIO; // driven/driving
	private static final double TURN_GEAR_RATIO = 1 / L3_TURN_RATIO; // drive/driving

	private final TalonFX driveTalonFX;
	private final TalonFX turnTalonFX;
	private final TalonFXConfiguration driveConfig;
	private final TalonFXConfiguration turnConfig;
	private final TalonFXConfigurator driveConfigurator;
	private final TalonFXConfigurator turnConfigurator;

	private final CANcoder turnAbsoluteEncoder;

	private final boolean isTurnMotorInverted = true;

	public ModuleIOTalonFX(SwerveModuleConfiguration config) {

		driveTalonFX = new TalonFX(config.DRIVE_MOTOR, "Ducky");
		turnTalonFX = new TalonFX(config.TURN_MOTOR, "Ducky");

		driveTalonFX.setPosition(0);
		turnTalonFX.setPosition(0);

		driveConfigurator = driveTalonFX.getConfigurator();
		turnConfigurator = turnTalonFX.getConfigurator();

		turnAbsoluteEncoder = new CANcoder(config.ENCODER, "Ducky");

		driveConfigurator.apply(new TalonFXConfiguration());
		turnConfigurator.apply(new TalonFXConfiguration());

		driveConfig = new TalonFXConfiguration();
		turnConfig = new TalonFXConfiguration();

		driveTalonFX.clearStickyFaults();
		turnTalonFX.clearStickyFaults();
		turnAbsoluteEncoder.clearStickyFaults();

		driveConfig.Slot0.kP = MODULE_DRIVE_KP; // Drive kP
		driveConfig.Slot0.kV = MODULE_DRIVE_KF; // Drive FeedForwards

		// driveConfig.Feedback.SensorToMechanismRatio =
		//	DRIVE_GEAR_RATIO;

		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		driveConfig.CurrentLimits.withStatorCurrentLimit(120);

		driveConfigurator.apply(driveConfig);

		turnConfig.Slot0.kP = MODULE_TURN_KP;
		turnConfig.MotorOutput.Inverted =
				isTurnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		turnConfig.CurrentLimits.withStatorCurrentLimit(80);
		// turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		// turnConfig.CurrentLimits.withSupplyCurrentLimit(20); // Only needed when brownouts become an issue

		turnConfigurator.apply(turnConfig);

		driveTalonFX.setNeutralMode(NeutralModeValue.Brake);
		turnTalonFX.setNeutralMode(NeutralModeValue.Brake);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		inputs.drivePositionRad = -driveTalonFX.getPosition().getValueAsDouble() * 2 * PI / DRIVE_GEAR_RATIO;

		inputs.driveVelocityMetersPerSec =
				-driveTalonFX.getVelocity().getValueAsDouble() * 2 * PI / DRIVE_GEAR_RATIO * SWERVE_WHEEL_RAD;

		inputs.driveAppliedVolts = -driveTalonFX.getDutyCycle().getValueAsDouble()
				* driveTalonFX.getSupplyVoltage().getValueAsDouble();

		inputs.driveCurrentAmps = new double[] {driveTalonFX.getStatorCurrent().getValueAsDouble()};

		inputs.turnAbsolutePosition = Rotation2d.fromRotations(
				turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());

		inputs.turnAbsolutePositionRad = inputs.turnAbsolutePosition.getRadians();

		inputs.turnPosition = Rotation2d.fromRotations(turnTalonFX.getPosition().getValueAsDouble() / TURN_GEAR_RATIO);

		inputs.turnVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
						turnTalonFX.getVelocity().getValueAsDouble())
				/ TURN_GEAR_RATIO;

		inputs.turnAppliedVolts = turnTalonFX.getDutyCycle().getValueAsDouble()
				* turnTalonFX.getSupplyVoltage().getValueAsDouble();

		inputs.turnCurrentAmps = new double[] {turnTalonFX.getStatorCurrent().getValueAsDouble()};
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveTalonFX.setVoltage(-volts);
	}

	@Override
	public void setDriveVelocity(double mps) {
		driveTalonFX.set(-mps);
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnTalonFX.setVoltage(volts);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		driveTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		turnTalonFX.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
	}

	@Override
	public void logTargetState(ModuleIOInputs inputs, SwerveModuleState state, double compensatedVel) {
		inputs.targetRad = state.angle.getRadians();
		inputs.targetVel = state.speedMetersPerSecond;
		inputs.compensatedTargetVel = compensatedVel;
	}
}
