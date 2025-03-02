package frc.robot.subsystems.arm;

import static frc.robot.constants.Constants.ArmConstants.SHOULDER_1;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_2;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_3;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_4;
import static frc.robot.constants.Constants.ArmConstants.TELESCOPE_1;
import static frc.robot.constants.Constants.ArmConstants.TELESCOPE_2;
import static frc.robot.constants.Constants.ArmConstants.WRIST;
import static frc.robot.constants.Constants.SHOULDER_GEAR_RATIO;
import static frc.robot.constants.Constants.TELESCOPE_GEAR_RATIO;
import static frc.robot.constants.Constants.TELESCOPE_PULLEY_RADIUS;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmIOHardware implements ArmIO {

	private static double ROTATION_GEAR_RATIO = 1 / SHOULDER_GEAR_RATIO;
	private static double SHOULDER_ENCODER_RATIO = 8;

	private static double EXTENSION_GEAR_RATIO = 1 / TELESCOPE_GEAR_RATIO;
	private static double EXTENSION_SPROCKET_RADIUS = Units.inchesToMeters(1.273);

	private final DigitalInput bottomSwitch;
	// private final DigitalInput topSwitch;

	private final TalonFX shoulder1;
	private final TalonFX shoulder2;
	private final TalonFX shoulder3;
	private final TalonFX shoulder4;

	private final TalonFX telescope1;
	private final TalonFX telescope2;

	private final SparkFlex wrist;

	private final CANcoder shoulderEncoder;
	private final Canandmag wristEncoder;

	private double extension;
	private double init_angle;

	public ArmIOHardware() {

		bottomSwitch = new DigitalInput(0);

		shoulder1 = new TalonFX(SHOULDER_1);
		shoulder2 = new TalonFX(SHOULDER_2);
		shoulder3 = new TalonFX(SHOULDER_3);
		shoulder4 = new TalonFX(SHOULDER_4);

		TalonFXConfigurator s1Configurator = shoulder1.getConfigurator();
		TalonFXConfigurator s2Configurator = shoulder2.getConfigurator();
		TalonFXConfigurator s3Configurator = shoulder3.getConfigurator();
		TalonFXConfigurator s4Configurator = shoulder4.getConfigurator();

		telescope1 = new TalonFX(TELESCOPE_1);
		telescope2 = new TalonFX(TELESCOPE_2);

		TalonFXConfigurator t1Configurator = telescope1.getConfigurator();
		TalonFXConfigurator t2Configurator = telescope2.getConfigurator();

		TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
		shoulderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		shoulderConfig.CurrentLimits.StatorCurrentLimit = 60;

		s1Configurator.apply(shoulderConfig);
		s2Configurator.apply(shoulderConfig);
		s3Configurator.apply(shoulderConfig);

		s4Configurator.apply(shoulderConfig);

		shoulder2.setControl(new Follower(SHOULDER_1, false));
		shoulder3.setControl(new Follower(SHOULDER_1, true));
		shoulder4.setControl(new Follower(SHOULDER_1, true));

		shoulder1.setPosition(0);

		shoulder1.setNeutralMode(NeutralModeValue.Brake);
		shoulder2.setNeutralMode(NeutralModeValue.Brake);
		shoulder3.setNeutralMode(NeutralModeValue.Brake);
		shoulder4.setNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration telescopeConfig = new TalonFXConfiguration();
		telescopeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		telescopeConfig.CurrentLimits.StatorCurrentLimit = 40;

		t1Configurator.apply(telescopeConfig);
		t2Configurator.apply(telescopeConfig);

		telescope2.setControl(new Follower(telescope1.getDeviceID(), false));

		telescope1.setPosition(0);
		telescope2.setPosition(0);

		shoulderEncoder = new CANcoder(26);
		extension = 0;

		wrist = new SparkFlex(39, MotorType.kBrushless);
		wristEncoder = new Canandmag(WRIST);

		
		SparkMaxConfig wristConfig = new SparkMaxConfig();
		wristConfig.smartCurrentLimit(30).idleMode(IdleMode.kBrake);
	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		inputs.shoulderRotation = new Rotation2d(
				Units.rotationsToRadians(shoulderEncoder.getAbsolutePosition().getValueAsDouble()));

		inputs.shoulderPivotVoltage = shoulder1.getMotorVoltage().getValueAsDouble();

		inputs.shoulderPivotCurrentAmps =
				new double[] {shoulder1.getStatorCurrent().getValueAsDouble()};

		inputs.shoulderAngVel = Units.rotationsPerMinuteToRadiansPerSecond(
						shoulder1.getVelocity().getValueAsDouble() * 60)
				* SHOULDER_ENCODER_RATIO;

		inputs.telescopePosition = extension
				+ (Units.rotationsToRadians(telescope1.getPosition().getValueAsDouble())
						* EXTENSION_GEAR_RATIO
						* TELESCOPE_PULLEY_RADIUS);

		inputs.telescopeVelocity = Units.rotationsPerMinuteToRadiansPerSecond(
						telescope1.getVelocity().getValueAsDouble() * 60)
				* TELESCOPE_PULLEY_RADIUS
				* EXTENSION_GEAR_RATIO;

		inputs.telescopeVoltage = telescope1.getDutyCycle().getValueAsDouble()
				* telescope1.getSupplyVoltage().getValueAsDouble();

		inputs.telescopeCurrent = new double[] {telescope1.getStatorCurrent().getValueAsDouble()};

		// inputs.topSwitchOn = topSwitch.get();

		// TOOD: Add inputs for wrist
		inputs.wristRotation = Rotation2d.fromRotations(wristEncoder.getAbsPosition());
		inputs.wristPivotVoltage = wrist.getAppliedOutput();

	}

	@Override
	public void setShoulderVoltage(double voltage) {
		shoulder1.setVoltage(-voltage); // Negative since motor directions and encoder directions are opposite and this is an easier
		// fix
	}

	@Override
	public void setExtensionVoltage(double voltage) {
		telescope1.setVoltage(voltage);
	}

	@Override
	public void setWristVoltage(double voltage) {
		wrist.setVoltage(-voltage);
	}

	@Override
	public void setShoulderVoltageClamped(double voltage) {
		setShoulderVoltage(MathUtil.clamp(voltage, -4, 4));
	}

	@Override
	public void setExtensionVoltageClamped(double voltage) {
		setExtensionVoltage(MathUtil.clamp(voltage, -6, 6));
	}

	@Override
	public void setWristVoltageClamped(double voltage) {
		setWristVoltage(MathUtil.clamp(voltage, -2, 2));
	}
}
