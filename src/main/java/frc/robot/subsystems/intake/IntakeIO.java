package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

	public default void setVelocity(double velocity) {}
	;

	public default void setVoltage(double voltage) {}
	;

	public default void stopMotor() {}
	;

	public default void updateInputs(IntakeIOInputs inputs) {}
	;

	@AutoLog
	public static class IntakeIOInputs {
		public double velocity = 0.0;
		public double voltage = 0.0;
		public boolean beamBreakIsActive = false;
		public double[] current = {};
	}
}
