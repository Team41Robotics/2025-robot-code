package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.ArmConstants.MAX_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;

public class ArmSubsystem extends SubsystemBase {

	private ArmIOHardware io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private Optional<Rotation2d> shoulderTargetRotation = Optional.empty();
	private Optional<Rotation2d> wristTargetRotation = Optional.empty();
        private Optional<Double> targetExtension = Optional.empty();

	public ArmSubsystem() {
		io = new ArmIOHardware();
	}

	public void periodic() {
		io.updateInputs(inputs);

                if(!shoulderTargetRotation.isEmpty()){
                        io.setToShoulderTargetRotation(getShoulderAngle(), shoulderTargetRotation.get());
                        if(Math.abs(getShoulderAngle().minus(shoulderTargetRotation.get()).getRadians()) < 0.1){
                                System.out.println("REACHED TARGET ROTATION");
                                shoulderTargetRotation = Optional.empty();
                        }
                }
                if(!targetExtension.isEmpty()){
                        io.setToTargetExtension(clampTargetExtension(this.targetExtension.get()));
                        if(Math.abs(getExtension() - this.targetExtension.get()) < 0.01){
                                targetExtension = Optional.empty();
                        }
                }

                Logger.recordOutput("/Arm/Current Rotation", getShoulderAngle().getRadians());
                Logger.recordOutput("Arm/Current Extension", getExtension());
                Logger.recordOutput("Arm/Shoulder Voltage", inputs.shoulderPivotVoltage);
                Logger.recordOutput("Arm/Shoulder Velocity", inputs.shoulderAngVel);
                if(!shoulderTargetRotation.isEmpty()) Logger.recordOutput("Arm/Target Rotation", this.shoulderTargetRotation.get().getRadians());

	}
        
        public void zero(){
                setShoulderTargetRotation(MIN_ROTATION);
                setTargetExtension(MIN_EXTENSION);
        }

        public void setShoulderTargetRotation(Rotation2d rotation){
                Rotation2d adjusted_rotation = new Rotation2d(rotation.getRadians() * 8);
                this.shoulderTargetRotation = Optional.of(rotation);
        }

        public void setTargetExtension(double length){
                this.targetExtension = Optional.of(length);
        }


        public Rotation2d clampTargetAngle(Rotation2d target){
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
