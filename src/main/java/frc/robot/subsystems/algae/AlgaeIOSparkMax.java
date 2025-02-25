package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeIOSparkMax implements AlgaeIO{
        
        private final DutyCycleEncoder pivotEncoder;
        private final CANSparkMax algaeMotor;
        private final PIDController m_PID;

        public AlgaeIOSparkMax {
                pivotEncoder = new DutyCycleEncoder(1);
                algaeMotor = new CANSparkMax(0, null);
                m_PID = new PIDController(0,0,0)
        }
        @Override 
        public void updateInputs(AlgaeIOInputs inputs){
                
        }       
}
