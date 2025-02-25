package frc.robot.subsystems.arm;

import static java.lang.Math.PI;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_1;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_2;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_3;
import static frc.robot.constants.Constants.ArmConstants.SHOULDER_4;
import static frc.robot.constants.Constants.ArmConstants.TELESCOPE_1;
import static frc.robot.constants.Constants.ArmConstants.TELESCOPE_2;
import static frc.robot.constants.Constants.TELESCOPE_PULLEY_RADIUS;

public class ArmIOHardware implements ArmIO {

	private static double SHOULDER_VELOCITY_RATIO = (54/18) * (54/18) * (72/9); // Full Gear ratio 
	private static double SHOULDER_ENCODER_RATIO = 8; // Encoder sprocket ratoi
        private static double SHOULDER_MAX_VELOCITY = Units.radiansPerSecondToRotationsPerMinute((Math.PI*1.75)*SHOULDER_VELOCITY_RATIO)/60; // Rotations per second 
        private static double SHOULDER_MAX_ACCELERATION = Units.radiansPerSecondToRotationsPerMinute(Math.PI) / 60; // Rotations per second^2

        private static double EXTENSION_GEAR_RATIO; // TODO: figure out what these are from 'thew
	private static double EXTENSION_SPROCKET_RADIUS;
        private static double EXTENSION_MAX_VELOCITY;
        private static double EXETNSION_MAX_ACCELERATION;

	// private final DigitalInput bottomSwitch;
	// private final DigitalInput topSwitch;

	private final TalonFX shoulder1;
	private final TalonFX shoulder2;
	private final TalonFX shoulder3;
	private final TalonFX shoulder4;

	private final TalonFX telescope1;
	private final TalonFX telescope2;

       // private final SparkMax wrist;

	// private final CANcoder shoulderEncoder;

	private final PIDController shoulderPID;
	private final PIDController telescopePID;
	private final PIDController wristPID;

	// private final SparkFlex wrist;
	private double extension;

	public ArmIOHardware() {

		//bottomSwitch = new DigitalInput(0);
		//topSwitch = new DigitalInput(0);

		// if (bottomSwitch.get()) {
			//extension = 0;
		// }
		// if (topSwitch.get()) {
			//extension = MAX_ARM_EXTEND;
		// }

		shoulder1 = new TalonFX(SHOULDER_1);
		shoulder2 = new TalonFX(SHOULDER_2);
		shoulder3 = new TalonFX(SHOULDER_3);
		shoulder4 = new TalonFX(SHOULDER_4);

		TalonFXConfigurator s1Configurator = shoulder1.getConfigurator();
		TalonFXConfigurator s2Configurator = shoulder2.getConfigurator();
		TalonFXConfigurator s3Configurator = shoulder3.getConfigurator();
		TalonFXConfigurator s4Configurator = shoulder4.getConfigurator();

		shoulderPID = new PIDController(0.2,0,0);

		telescope1 = new TalonFX(TELESCOPE_1);
		telescope2 = new TalonFX(TELESCOPE_2);

		TalonFXConfigurator t1Configurator = telescope1.getConfigurator();
		TalonFXConfigurator t2Configurator = telescope2.getConfigurator();

		telescopePID = new PIDController(0,0,0);
		//wrist = new SparkMax(WRIST, MotorType.kBrushless);

		wristPID = new PIDController(0, 0, 0);

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
		telescopeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		telescopeConfig.CurrentLimits.StatorCurrentLimit = 30;

		t1Configurator.apply(telescopeConfig);
		t2Configurator.apply(telescopeConfig);

		telescope2.setControl(new Follower(telescope1.getDeviceID(), false));

		telescope1.setPosition(0);
		telescope2.setPosition(0);

		// shoulderEncoder = new CANcoder(0);

	}

	@Override
	public void updateInputs(ArmIOInputs inputs) {
		inputs.shoulderRotation =
				new Rotation2d(shoulder1.getPosition().getValueAsDouble() * SHOULDER_VELOCITY_RATIO *2*PI);

		inputs.shoulderPivotVoltage = shoulder1.getMotorVoltage().getValueAsDouble();

		inputs.shoulderPivotCurrentAmps =
				new double[] {shoulder1.getStatorCurrent().getValueAsDouble()};

		inputs.shoulderAngVel = shoulder1.getVelocity().getValueAsDouble() * SHOULDER_ENCODER_RATIO;

		inputs.telescopePosition = extension + telescope1.getVelocity().getValueAsDouble() * TELESCOPE_PULLEY_RADIUS;

		inputs.telescopeVelocity = telescope1.getVelocity().getValueAsDouble() * TELESCOPE_PULLEY_RADIUS;

		inputs.telescopeVoltage = telescope1.getDutyCycle().getValueAsDouble()
				* telescope1.getSupplyVoltage().getValueAsDouble();

		inputs.telescopeCurrent = new double[] {telescope1.getStatorCurrent().getValueAsDouble()};

		// inputs.bottomSwitchOn = bottomSwitch.get();

		// inputs.topSwitchOn = topSwitch.get();               

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
        public void setToShoulderTargetRotation(Rotation2d current, Rotation2d target) {
		double out = shoulderPID.calculate(current.getRadians(), target.getRadians());
		MathUtil.clamp(out, -1,1);
		setShoulderVoltage(out);
	}

        @Override
	public void setToTargetExtension(double extension) {
		double out = telescopePID.calculate(extension);
		MathUtil.clamp(out, -1, 1);
		setExtensionVoltage(out);
        }                                                             	



}
        