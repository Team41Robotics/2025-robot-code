package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_1;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_2;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_3;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_4;
import static frc.robot.constants.Constants.ArmConstants.TELESCOPE_1;
import static frc.robot.constants.Constants.ArmConstants.TELESCOPE_2;
import static frc.robot.constants.Constants.MAX_ARM_EXTEND;
import static frc.robot.constants.Constants.TELESCOPE_PULLEY_RADIUS;

public class ArmIOHardware implements ArmIO {

	private static double SHOULDER_ANGLE_CONST = (9 / 1); // Gear ratio 
        private static double SHOULDER_MAX_VELOCITY = Units.radiansPerSecondToRotationsPerMinute((Math.PI*1.75)*SHOULDER_ANGLE_CONST)/60; // Rotations per second 
        private static double SHOULDER_MAX_ACCELERATION = Units.radiansPerSecondToRotationsPerMinute(Math.PI) / 60; // Rotations per second^2
	private final DigitalInput bottomSwitch;
	private final DigitalInput topSwitch;

	private final TalonFX shoulder1;
	private final TalonFX shoulder2;
	private final TalonFX shoulder3;
	private final TalonFX shoulder4;

	private final TalonFX telescope1;
	private final TalonFX telescope2;

        private final SparkMax wrist;

	private final CANcoder shoulderEncoder;

	// private final SparkFlex wrist;
	private double extension;

	public ArmIOHardware() {

		bottomSwitch = new DigitalInput(0);
		topSwitch = new DigitalInput(0);

		if (bottomSwitch.get()) {
			extension = 0;
		}
		if (topSwitch.get()) {
			extension = MAX_ARM_EXTEND;
		}

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

		wrist = new SparkMax(0, null);

		TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();
		shoulderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		shoulderConfig.CurrentLimits.SupplyCurrentLimit = 25;
		shoulderConfig.Slot0.kP = 0.0;
		shoulderConfig.Slot0.kI = 0.0;
		shoulderConfig.Slot0.kD = 0.0;
                shoulderConfig.MotionMagic.MotionMagicCruiseVelocity = SHOULDER_MAX_VELOCITY * SHOULDER_ANGLE_CONST; 
                shoulderConfig.MotionMagic.MotionMagicAcceleration = SHOULDER_MAX_ACCELERATION * SHOULDER_ANGLE_CONST;

		s1Configurator.apply(shoulderConfig);
		s2Configurator.apply(shoulderConfig);
		s3Configurator.apply(shoulderConfig);
		s4Configurator.apply(shoulderConfig);

		shoulder2.setControl(new Follower(shoulder1.getDeviceID(), false)); // TODO: Configure correctly
		shoulder3.setControl(new Follower(shoulder1.getDeviceID(), false));
		shoulder4.setControl(new Follower(shoulder1.getDeviceID(), false));

		shoulder1.setNeutralMode(NeutralModeValue.Brake);
		shoulder2.setNeutralMode(NeutralModeValue.Brake);
		shoulder3.setNeutralMode(NeutralModeValue.Brake);
		shoulder4.setNeutralMode(NeutralModeValue.Brake);

		TalonFXConfiguration telescopeConfig = new TalonFXConfiguration();
		telescopeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		telescopeConfig.CurrentLimits.StatorCurrentLimit = 30;
                telescopeConfig.Slot0.kS = 0.25;
                telescopeConfig.Slot0.kV = 0.12;
                telescopeConfig.Slot0.kA = 0.01;
		telescopeConfig.Slot0.kP = 4.8; // TODO: Figure these out 
		telescopeConfig.Slot0.kI = 0.0;
		telescopeConfig.Slot0.kD = 0.1;

		t1Configurator.apply(telescopeConfig);
		t2Configurator.apply(telescopeConfig);

		telescope2.setControl(new Follower(telescope1.getDeviceID(), false));

		telescope1.setPosition(0);
		telescope2.setPosition(0);

		shoulderEncoder = new CANcoder(0);
	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		inputs.shoulderRotation =
				new Rotation2d(shoulderEncoder.getAbsolutePosition().getValueAsDouble() * SHOULDER_ANGLE_CONST);

		inputs.shoulderPivotVoltage = shoulder1.getDutyCycle().getValueAsDouble()
				* shoulder1.getSupplyVoltage().getValueAsDouble();

		inputs.shoulderPivotCurrentAmps =
				new double[] {shoulder1.getStatorCurrent().getValueAsDouble()};

		inputs.shoulderAngVel = shoulderEncoder.getVelocity().getValueAsDouble() * SHOULDER_ANGLE_CONST;

		inputs.telescopePosition = extension + telescope1.getVelocity().getValueAsDouble() * TELESCOPE_PULLEY_RADIUS;

		inputs.telescopeVelocity = telescope1.getVelocity().getValueAsDouble() * TELESCOPE_PULLEY_RADIUS;

		inputs.telescopeVoltage = telescope1.getDutyCycle().getValueAsDouble()
				* telescope1.getSupplyVoltage().getValueAsDouble();

		inputs.telescopeCurrent = new double[] {telescope1.getStatorCurrent().getValueAsDouble()};

		inputs.bottomSwitchOn = bottomSwitch.get();

		inputs.topSwitchOn = topSwitch.get();

		// TOOD: Add inputs for wrist

	}


	@Override
	public void setShoulderVoltage(double voltage) {
		shoulder1.setVoltage(voltage); // Gotta love motor following :D
	}

	@Override
	public void setExtensionVoltage(double voltage) {
		telescope1.setVoltage(voltage);
	}

        @Override
        public void setToShoulderTargetRotation(Rotation2d target) {
                MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
                shoulder1.setControl(m_request.withVelocity(target.getRadians() * SHOULDER_ANGLE_CONST));
	}

        @Override
	public void setToTargetExtension(double extension) {
                MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
                shoulder1.setControl(m_request.withVelocity(0));
        }
}
        