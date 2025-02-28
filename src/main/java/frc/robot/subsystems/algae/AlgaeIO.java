package frc.robot.subsystems.algae;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {

	public default void setToAlgaeTargetRotation(Rotation2d target) {}
	;

	public default void setAlgaeVoltage(double voltage) {}
	;

	public default void updateInputs(AlgaeIOInputs inputs) {}

	@AutoLog
	public static class AlgaeIOInputs {

		public Rotation2d algaeRotation = new Rotation2d();
		public double algaePivotVoltage = 0.0;
		public double[] algaePivotCurrent = new double[] {}; // amps
		public double algaeAngVel = 0.0; // rad / s
	}
}
