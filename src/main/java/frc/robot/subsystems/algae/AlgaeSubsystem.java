package frc.robot.subsystems.algae;

import java.util.Optional;

public class AlgaeSubsystem extends SubsystemBase {
        private AlgaeIOSparkMax io;
        private IntakeIOInputsAutoLogged inputs;
        private PIDController m_PID;
        private Optional<Rotation2d> targetRotation;


        public AlgaeSubsystem {
            io = new AlgaeIOSparkMax();
            inputs = new AlgaeIOInputsAutoLogged();
            targetRotation = Optional.empty();
            m_PID = new PIDController(0,0,0); //todo
        }

        @Override
        public void periodic(){
            io.updateInputs(inputs);
            if(!targetRotation.isEmpty()){
                double out = m_PID.calculate(inputs.algaeRotation, clampTargetAngle(targetRotation));
                io.setAlgaeVoltage(out);
            }
        }
        public Rotation2d clampTargetAngle(Rotation2d target){
            target = new Rotation2d(MathUtil.clamp(target.getRadians(), 0, Math.PI));
            return target;
        }
}
