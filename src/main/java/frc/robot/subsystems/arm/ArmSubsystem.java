package frc.robot.subsystems.arm;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

	private ArmIOHardware io;
	private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

	private Optional<Rotation2d> shoulderTargetRotation;
	private Optional<Rotation2d> wristTargetRotation;
        private Optional<Double> targetExtension;

	public ArmSubsystem() {
		io = new ArmIOHardware();
	}

	public void periodic() {
		io.updateInputs(inputs);

                if(!shoulderTargetRotation.isEmpty()){
                        Rotation2d target = getShoulderAngle().minus(shoulderTargetRotation.get());
                        io.setToShoulderTargetRotation(target);
                }
                if(!targetExtension.isEmpty()){

                }
	}

	public double getExtension() {
		return inputs.telescopePosition;
	}

        public Rotation2d getShoulderAngle(){
                return inputs.shoulderRotation;
        }
}
