package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface AlgaeIO {

	public default void setAlgaeVoltage(double voltage) {}

	public default void setIntakeVoltage(double voltage) {}

	public default void updateInputs(AlgaeIOInputs inputs) {}



	@AutoLog
	public static class AlgaeIOInputs {

		public Rotation2d algaeRotation = new Rotation2d();
		public Rotation2d algaeRotationAbsolute = new Rotation2d();
		public double algaePivotVoltage = 0.0;
		public double[] algaePivotCurrent = new double[] {}; // amps
		public double algaeAngVel = 0.0; // rad / s

		public double intakeVoltage = 0.0;
		public double intakeSpeed = 0.0;
		public double[] intakeCurrent = new double[]{};

	}
}
