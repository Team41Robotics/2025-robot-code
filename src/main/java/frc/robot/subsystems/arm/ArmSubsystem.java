package frc.robot.subsystems.arm;

import static frc.robot.RobotContainer.robot;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;
import static frc.robot.util.Util.ramp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {

	private final ArmIOHardware io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private double shoulderTargetRotation = 0;

	private double wristTargetRotation = 4.83;
	private double targetExtension = 0;

	private final PIDController shoulderPID;
	private final PIDController telescopePID;
	private final PIDController wristPID;

	double shoulder_ramped, ext_ramped, wrist_ramped;

	public ArmSubsystem() {
		io = new ArmIOHardware();
		shoulderPID = new PIDController(7, 1, 0.);
		shoulderPID.setIZone(0.2);
		telescopePID = new PIDController(6, 0, 0);
		telescopePID.setTolerance(0.1);
		telescopePID.setIZone(0.1);
		wristPID = new PIDController(6, 1.2, 0);
		// wristPID.setIZone(0.2);
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
			ext_ramped = ramp(targetExtension, ext_ramped, 0.85);
			double out = telescopePID.calculate(getExtension(), ext_ramped);
			io.setExtensionVelocity(out);
		}
		{
			wristTargetRotation = clampWristTargetAngle(wristTargetRotation);
			wrist_ramped = ramp(wristTargetRotation, wrist_ramped, 3.);
			double out = wristPID.calculate(inputs.wristRotation, wrist_ramped);
			io.setWristVoltageClamped(out);
			Logger.recordOutput("Arm/Output", out);
		}

		Logger.processInputs("Arm", inputs);
		Logger.recordOutput("Arm/Target Extension", this.targetExtension);
		Logger.recordOutput("Arm/Target Rotation", this.shoulderTargetRotation);
		Logger.recordOutput("Arm/Target Wrist Rotation", this.wristTargetRotation);
		Logger.recordOutput("Arm/Error Wrist", this.wristTargetRotation - this.getWristAngle());
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
		return MathUtil.clamp(target, 0.74, 5.16);
	}

	public double getExtension() {
		return inputs.telescopePosition;
	}

	public Rotation2d getShoulderAngle() {
		return inputs.shoulderRotation;
	}

	public double getShoulderAngleRadians() {
		return inputs.shoulderRotation.getRadians();
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

	public void setWristVoltage(double voltage) {
		io.setWristVoltage(voltage);
	}
}
