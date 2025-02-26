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

        private final SysIdRoutine routine;        


	public ArmSubsystem() {
		io = new ArmIOHardware();
                shoulderPID = new PIDController(4,0,0);
		telescopePID = new PIDController(1,0,0);
                routine = new SysIdRoutine(
                       new SysIdRoutine.Config(null, Voltage.ofBaseUnits(4, Volts), null), 
                       new SysIdRoutine.Mechanism(
                        (voltage) -> {
                                if(getShoulderAngle().getRadians() < 0.174  || getShoulderAngle().getRadians() > Math.PI/2)
                                        io.setShoulderVoltage(0);
                                else
                                        io.setShoulderVoltageClamped(voltage.baseUnitMagnitude());
                        },
                         log -> {
                                log.motor("Shoulder Motor")
                                .voltage(Volts.mutable(0).mut_replace(inputs.shoulderPivotVoltage, Volts))
                                .angularPosition(Radians.mutable(0).mut_replace(inputs.shoulderRotation.getRadians(), Radians))
                                .angularVelocity(RadiansPerSecond.mutable(0).mut_replace(inputs.shoulderAngVel, RadiansPerSecond));
                        },
                          this,
                           "Arm")       
                );
	}

	public void periodic() {
		io.updateInputs(inputs);

                if(false) {
                        if(!shoulderTargetRotation.isEmpty()){
                                shoulderTargetRotation = Optional.of(clampShoulderTargetAngle(shoulderTargetRotation.get()));
                                double out = shoulderPID.calculate(getShoulderAngle().getRadians(), shoulderTargetRotation.get().getRadians());
                                io.setShoulderVoltageClamped(out);
                                
                        }
                        if(!targetExtension.isEmpty()){
                                targetExtension = Optional.of(clampTargetExtension(targetExtension.get())); // Ik its cursed ignore it
                                double out = telescopePID.calculate(getExtension(), getExtension()+((targetExtension.get())/(TELESCOPE_PULLEY_RADIUS * (1/TELESCOPE_GEAR_RATIO))));
                                io.setExtensionVoltageClamped(out);
                        }
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

        public Command sysIDQuasistatic(SysIdRoutine.Direction direction){
                return routine.quasistatic(direction);
        }

        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
                return routine.dynamic(direction);
        }
              
}
