package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOSparkMax implements IntakeIO {
	private final SparkFlex m_motor;

	private final PIDController m_PID;

	private final DigitalInput beamBreak;

	public IntakeIOSparkMax() {
		m_PID = new PIDController(1, 0, 0); // TODO
		m_motor = new SparkFlex(41, MotorType.kBrushless);
		beamBreak = new DigitalInput(6);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {
		inputs.velocity = m_motor.get();
		inputs.voltage = m_motor.getBusVoltage();
		inputs.current = new double[] {m_motor.getOutputCurrent()};
		inputs.beamBreakStatus = beamBreak.get();
	}

	@Override
	public void setVelocity(double velocity) {
		m_motor.set(velocity);
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
