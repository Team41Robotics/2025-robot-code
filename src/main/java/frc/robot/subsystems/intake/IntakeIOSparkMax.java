package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
	private final SparkMax m_motor;

	private final PIDController m_PID;

	private final DigitalInput beamBreak;

	public IntakeIOSparkMax() {
		m_PID = new PIDController(0, 0, 0); // TODO
		m_motor = new SparkMax(0, MotorType.kBrushless);
		beamBreak = new DigitalInput(7);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.velocity = m_motor.get();
		inputs.voltage = m_motor.getBusVoltage();
		inputs.current = new double[] {m_motor.getOutputCurrent()};
		inputs.beamBreakIsActive = beamBreak.get();
	}

	@Override
	public void setVelocity(double velocity) {
		double out = m_PID.calculate(velocity);
		setVoltage(out);
	}

	@Override
	public void setVoltage(double voltage) {
		m_motor.setVoltage(voltage);
	}

	@Override
	public void stopMotor() {
		m_motor.set(0);
	}
}
