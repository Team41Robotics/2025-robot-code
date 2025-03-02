package frc.robot.subsystems.arm;

import static java.lang.Math.PI;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;
import static frc.robot.util.Util.rampVoltage;

public class ArmSubsystem extends SubsystemBase {

	private final ArmIOHardware io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private Optional<Rotation2d> shoulderTargetRotation = Optional.empty();
	private Optional<Double> wristTargetRotation = Optional.empty();
	private Optional<Double> targetExtension = Optional.empty();

	private final PIDController shoulderPID;
	private final PIDController telescopePID;
	private final PIDController wristPID;

	private double shoulder_previous_voltage;
	//private double extension_previous_voltage;
	private double wrist_previous_voltage;

	public ArmSubsystem() {
		io = new ArmIOHardware();
		shoulderPID = new PIDController(3, 0, 0);
		telescopePID = new PIDController(13, 4, 0);
		telescopePID.setTolerance(0.1);
		wristPID = new PIDController(2, 0, 0.15);
		shoulder_previous_voltage = 0.0;
		//extension_previous_voltage = 0.0;
		wrist_previous_voltage = 0.0;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);

		if (!shoulderTargetRotation.isEmpty()) {
			shoulderTargetRotation = Optional.of(clampShoulderTargetAngle(shoulderTargetRotation.get()));
			double out = shoulderPID.calculate(
					getShoulderAngle().getRadians(),
					shoulderTargetRotation.get().getRadians());
			io.setShoulderVoltageClamped(rampVoltage(out, shoulder_previous_voltage));
			shoulder_previous_voltage = out;
			if (shoulderPID.atSetpoint()) {
				shoulder_previous_voltage = 0.;
			}
		}
		if (!targetExtension.isEmpty()) {
			targetExtension = Optional.of(clampTargetExtension(targetExtension.get())); // Ik its cursed ignore it
			double out = telescopePID.calculate(getExtension(), targetExtension.get());
			io.setExtensionVoltageClamped(out);
		}	
		if (!wristTargetRotation.isEmpty()) {
			wristTargetRotation = Optional.of(clampWristTargetAngle(wristTargetRotation.get()));
			double out = wristPID.calculate(
					inputs.wristRotation, wristTargetRotation.get());
			io.setWristVoltageClamped(rampVoltage(out, wrist_previous_voltage));
			wrist_previous_voltage = out;
			if (wristPID.atSetpoint()) {

				wrist_previous_voltage = 0.;
			}
		}

		updateLogging();
	}

	public void updateLogging() {
		Logger.processInputs("Arm", inputs);
		Logger.recordOutput("/Arm/Current Rotation", getShoulderAngle().getRadians());
		Logger.recordOutput("Arm/Current Extension", getExtension());
		Logger.recordOutput("Arm/Shoulder Voltage", inputs.shoulderPivotVoltage);
		Logger.recordOutput("Arm/Shoulder Velocity", inputs.shoulderAngVel);
		Logger.recordOutput("Arm/Current Extension", getExtension());
		Logger.recordOutput("Arm/Extension Velocity", inputs.telescopeVelocity);
		Logger.recordOutput("Arm/Extension Voltage", inputs.telescopeVoltage);
		Logger.recordOutput("Arm/Extension Current", inputs.telescopeCurrent);
		Logger.recordOutput("Arm/Current Wrist Rotation", inputs.wristRotation);
		Logger.recordOutput("Arm/Wrist Voltage", inputs.wristPivotVoltage);
		if (!targetExtension.isEmpty()) Logger.recordOutput("Arm/Target Extension", this.targetExtension.get());
		if (!shoulderTargetRotation.isEmpty())
			Logger.recordOutput(
					"Arm/Target Rotation", this.shoulderTargetRotation.get().getRadians());
		if(!wristTargetRotation.isEmpty()) Logger.recordOutput("Arm/Target Wrist Rotation", this.wristTargetRotation.get());
	}

	public void zero() {
		setShoulderTargetRotation(MIN_ROTATION);
		setTargetExtension(MIN_EXTENSION);
	}

	public void setShoulderTargetRotation(Rotation2d rotation) {
		this.shoulderTargetRotation = Optional.of(rotation);
	}

	public void setTargetExtension(double length) {
		this.targetExtension = Optional.of(length);
	}

	public void setWristTargetRotation(double rotation) {
		System.out.println("UPDATED TARGET");
		this.wristTargetRotation = Optional.of(rotation);
	}

	public Rotation2d clampShoulderTargetAngle(Rotation2d target) {
		target = new Rotation2d(MathUtil.clamp(target.getRadians(), 0.175, Math.PI / 2));
		return target;
	}

	
	public double clampTargetExtension(double extension) {
		return MathUtil.clamp(extension, MIN_EXTENSION, MAX_EXTENSION);
	}

	public double clampWristTargetAngle(double target){
		return MathUtil.clamp(target, PI/4, 3*PI/2);
	}

	public double getExtension() {
		return inputs.telescopePosition;
	}

	public Rotation2d getShoulderAngle() {
		return inputs.shoulderRotation;
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
