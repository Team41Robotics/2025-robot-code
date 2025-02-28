package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeIOSparkMax implements AlgaeIO {

	private final DutyCycleEncoder pivotEncoder;
	private final SparkMax algaeMotor;
	private final PIDController m_PID;

	public AlgaeIOSparkMax() {
		pivotEncoder = new DutyCycleEncoder(1);
		algaeMotor = new SparkMax(0, null);
		m_PID = new PIDController(0, 0, 0);
	}

	@Override
	public void updateInputs(AlgaeIOInputs inputs) {}
}
