package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;
import static frc.robot.util.Util.rampVoltage;
import static frc.robot.constants.Constants.TELESCOPE_GEAR_RATIO;
import static frc.robot.constants.Constants.TELESCOPE_PULLEY_RADIUS;

public class ArmSubsystem extends SubsystemBase {

	private final ArmIOHardware io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private Optional<Rotation2d> shoulderTargetRotation = Optional.empty();
	private Optional<Rotation2d> wristTargetRotation = Optional.empty();
        private Optional<Double> targetExtension = Optional.empty();
        private final PIDController shoulderPID;
        private final PIDController telescopePID;
        private double shoulder_previous_voltage;
        private double extension_previous_voltage;


	public ArmSubsystem() {
		io = new ArmIOHardware();
                shoulderPID = new PIDController(4,0,0);
		telescopePID = new PIDController(1,0,0);
                shoulder_previous_voltage = 0.0;

	}

	public void periodic() {
		io.updateInputs(inputs);
                
                if(!shoulderTargetRotation.isEmpty()){
                        shoulderTargetRotation = Optional.of(clampShoulderTargetAngle(shoulderTargetRotation.get()));
                        double out = shoulderPID.calculate(getShoulderAngle().getRadians(), shoulderTargetRotation.get().getRadians());
                        io.setShoulderVoltageClamped(rampVoltage(out, shoulder_previous_voltage));
                        shoulder_previous_voltage = out;
                        if(shoulderPID.atSetpoint()){
                                shoulder_previous_voltage = 0.;
                        }
                }
                if(!targetExtension.isEmpty()){
                        targetExtension = Optional.of(clampTargetExtension(targetExtension.get())); // Ik its cursed ignore it
                        double out = telescopePID.calculate(getExtension(), getExtension()+((targetExtension.get())/(TELESCOPE_PULLEY_RADIUS * (1/TELESCOPE_GEAR_RATIO))));                                io.setExtensionVoltageClamped(out);
                }
                

                updateLogging();

	}

        public void updateLogging(){
                Logger.processInputs("Arm", inputs);
                Logger.recordOutput("/Arm/Current Rotation", getShoulderAngle().getRadians());
                Logger.recordOutput("Arm/Current Extension", getExtension());
                Logger.recordOutput("Arm/Shoulder Voltage", inputs.shoulderPivotVoltage);
                Logger.recordOutput("Arm/Shoulder Velocity", inputs.shoulderAngVel);
                Logger.recordOutput("Arm/Current Extension", getExtension());
                if(!targetExtension.isEmpty()) Logger.recordOutput("Arm/Target Extension", this.targetExtension.get());
                if(!shoulderTargetRotation.isEmpty()) Logger.recordOutput("Arm/Target Rotation", this.shoulderTargetRotation.get().getRadians());

        }
        
        public void zero(){
                setShoulderTargetRotation(MIN_ROTATION);
                setTargetExtension(MIN_EXTENSION);
        }

        public void setShoulderTargetRotation(Rotation2d rotation){
                this.shoulderTargetRotation = Optional.of(rotation);
        }

        public void setTargetExtension(double length){
                this.targetExtension = Optional.of(length);
        }


        public Rotation2d clampShoulderTargetAngle(Rotation2d target){
                target = new Rotation2d(MathUtil.clamp(target.getRadians(), 0, Math.PI/2));
                return target;
        }

        public double clampTargetExtension(double extension){
                return MathUtil.clamp(extension, MIN_EXTENSION, MAX_EXTENSION);
        }

        public double getExtension() {
		return inputs.telescopePosition;
	}

        public Rotation2d getShoulderAngle(){
                return inputs.shoulderRotation;
        }

}
