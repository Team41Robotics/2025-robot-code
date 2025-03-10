package frc.robot.subsystems.arm;

import static java.lang.Math.PI;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.robot;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;
import static frc.robot.util.Util.ramp;

public class ArmSubsystem extends SubsystemBase {

	private final ArmIOHardware io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private double shoulderTargetRotation = 0;
	private double wristTargetRotation = 4.4;
	private double targetExtension = 0;

	private final PIDController shoulderPID;
	private final PIDController telescopePID;
	private final PIDController wristPID;

	double shoulder_ramped, ext_ramped, wrist_ramped;
	// private double shoulder_previous_voltage;
	// private double extension_previous_voltage;
	// private double wrist_previous_voltage;

	public ArmSubsystem() {
		io = new ArmIOHardware();
		shoulderPID = new PIDController(7, 1, 0.);
		// shoulderPID = new PIDController(7, 0, 0.);
		shoulderPID.setIZone(0.2);
		telescopePID = new PIDController(30, 25, 0);
		telescopePID.setTolerance(0.1);
		telescopePID.setIZone(0.1);
		wristPID = new PIDController(3, 0.0, 0);
		// shoulder_previous_voltage = 0.0;
		// extension_previous_voltage = 0.0;
		// wrist_previous_voltage = 0.0;
	}

	@Override
	public void periodic() {
		if (robot.isDisabled()) {
			shoulder_ramped = getShoulderAngle().getRadians();
			ext_ramped = getExtension();
			wrist_ramped = getWristAngle();
		}
		io.updateInputs(inputs);

		{
			shoulderTargetRotation = clampShoulderTargetAngle(shoulderTargetRotation);
			shoulder_ramped = ramp(shoulderTargetRotation, shoulder_ramped, 1.);
			double out = shoulderPID.calculate(getShoulderAngle().getRadians(), shoulder_ramped);

			io.setShoulderVoltageClamped(out);
		}
		{
			targetExtension = clampTargetExtension(targetExtension);
			ext_ramped = ramp(targetExtension, ext_ramped, 0.25);
			double out = telescopePID.calculate(getExtension(), targetExtension);
			io.setExtensionVoltageClamped(out);
		}
		{
			wristTargetRotation = clampWristTargetAngle(wristTargetRotation);
			wrist_ramped = ramp(wristTargetRotation, wrist_ramped, 1);
			double out = wristPID.calculate(inputs.wristRotation, wrist_ramped);
			io.setWristVoltageClamped(out);
		}
		Logger.processInputs("Arm", inputs);
		Logger.recordOutput("Arm/Target Extension", this.targetExtension);
		Logger.recordOutput("Arm/Target Rotation", this.shoulderTargetRotation);
		Logger.recordOutput("Arm/Target Wrist Rotation", this.wristTargetRotation);
	}

	public void zero() {
		setShoulderTargetRotation(MIN_ROTATION);
		setTargetExtension(MIN_EXTENSION);
	}

	public void setShoulderTargetRotation(Rotation2d rotation) {
		this.shoulderTargetRotation = rotation.getRadians();
	}

	public void setTargetExtension(double length) {
		this.targetExtension = length;
	}

	public void setWristTargetRotation(double rotation) {
		this.wristTargetRotation = rotation;
	}

	public double clampShoulderTargetAngle(double target) {
		target = MathUtil.clamp(target, 0.175, Math.PI / 2);
		return target;
	}

	public double clampTargetExtension(double extension) {
		return MathUtil.clamp(extension, MIN_EXTENSION, MAX_EXTENSION);
	}

	public double clampWristTargetAngle(double target) {
		return MathUtil.clamp(target, PI / 4, 3 * PI / 2);
	}

	public double getExtension() {
		return inputs.telescopePosition;
	}

	public Rotation2d getShoulderAngle() {
		return inputs.shoulderRotation;
	}

	public double getWristAngle() {
		return inputs.wristRotation;
	}

	public boolean shoulderAtSetpoint() {
		return shoulderPID.atSetpoint();
	}

	public boolean extensionAtSetpoint() {
		return telescopePID.atSetpoint();
	}

	public boolean wristAtSetpoint() {
		return wristPID.atSetpoint();
	}
}
