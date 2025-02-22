package frc.robot.subsystems.arm;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.constants.Constants.ArmConstants.MIN_EXTENSION;
import static frc.robot.constants.Constants.ArmConstants.MIN_ROTATION;

public class ArmSubsystem extends SubsystemBase {

	private ArmIOHardware io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private Optional<Rotation2d> shoulderTargetRotation;
	private Optional<Rotation2d> wristTargetRotation;
        private Optional<Double> targetExtension;

	public ArmSubsystem() {
		io = new ArmIOHardware();
	}
  
        // TODO Setup motion model for arm, talk to Matthew and Jason about math (jason if u are reading this, show up more you bum)

	public void periodic() {
		io.updateInputs(inputs);

                if(!shoulderTargetRotation.isEmpty()){
                        io.setToShoulderTargetRotation(clampTargetAngle(this.shoulderTargetRotation.get()));
                }
                if(!targetExtension.isEmpty()){
                        io.setToTargetExtension(this.targetExtension.get());
                }

                Logger.recordOutput("/Arm/Current Rotation", getShoulderAngle().getRadians());
                Logger.recordOutput("Arm/Current Extension", getExtension());

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

	public double getExtension() {
		return inputs.telescopePosition;
	}

        public Rotation2d getShoulderAngle(){
                return inputs.shoulderRotation;
        }

        public Rotation2d clampTargetAngle(Rotation2d target){
                target = new Rotation2d(MathUtil.clamp(target.getRadians(), 0, Math.PI/2));
                return target;
        }

}
