package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeIOSparkMax implements AlgaeIO {

	private final CANcoder pivotEncoder;
	private final TalonFX intakeMotor;
	private final SparkMax pivotMotor;

	public AlgaeIOSparkMax() {
		pivotEncoder = new CANcoder(28);
		intakeMotor = new TalonFX(27);
		pivotMotor = new SparkMax(42, MotorType.kBrushless);

		SparkMaxConfig config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).smartCurrentLimit(60);
		TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
		TalonFXConfigurator intakeConfigurator = intakeMotor.getConfigurator();

		intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		intakeConfig.CurrentLimits.StatorCurrentLimit = 60;

		pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void updateInputs(AlgaeIOInputs inputs) {
		inputs.algaeRotation = new Rotation2d(MathUtil.angleModulus(pivotEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI));
		inputs.algaePivotVoltage = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
		inputs.algaeAngVel = pivotMotor.getEncoder().getVelocity() * 2 * Math.PI / 60.0;
		inputs.algaePivotCurrent = new double[] {pivotMotor.getOutputCurrent()};

		inputs.intakeSpeed = Units.rotationsPerMinuteToRadiansPerSecond(
						intakeMotor.getVelocity().getValueAsDouble())
				/ 60;
		inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
		inputs.intakeCurrent = new double[] {intakeMotor.getStatorCurrent().getValueAsDouble()};
	}

	@Override
	public void setAlgaeVoltage(double voltage) {
		// System.out.println(voltage);
		pivotMotor.setVoltage(-MathUtil.clamp(voltage, -3, 3));
	}

	@Override
	public void setIntakeVoltage(double voltage) {
		intakeMotor.setVoltage(MathUtil.clamp(voltage, -5, 5));
	}

	public void setIntakeVelocity(double velocity) {
		intakeMotor.set(velocity);
	}
}
