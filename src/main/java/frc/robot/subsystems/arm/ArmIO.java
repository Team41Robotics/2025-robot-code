package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

	public default void setWristVoltage(double voltage) {}
	;

	public default void setExtensionVoltage(double voltage) {}
	;

	public default void setShoulderVoltageClamped(double voltage) {}
	;

	public default void setExtensionVoltageClamped(double voltage) {}
	;

	public default void setWristVoltageClamped(double voltage) {}
	;

	public default void setShoulderVoltage(double voltage) {}
	;

	public default void updateInputs(ArmIOInputs inputs) {}
	;

	@AutoLog
	public static class ArmIOInputs {

		public Rotation2d shoulderRotation = new Rotation2d();
		public double shoulderPivotVoltage = 0.0;
		public double[] shoulderPivotCurrentAmps = new double[] {};
		public double shoulderAngVel = 0.0; // rads / s

		public double telescopePosition = 0.0; // m
		public double telescopeVelocity = 0.0; // m/s
		public double telescopeVoltage = 0.0;
		public double[] telescopeCurrent = new double[] {};

		public double wristRotation = 0.0;
		public double wristPivotVoltage = 0.0;
		public double[] wristPivotCurrent = new double[] {}; // amps
		public double wristAngVel = 0.0; // rad / s
	}
}
