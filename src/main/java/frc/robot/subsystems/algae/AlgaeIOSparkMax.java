package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeIOSparkMax implements AlgaeIO{
        
        DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);
        SparkMax algaeMotor = new SparkMax(0, null);

        @Override 
        public void updateInputs(AlgaeIOInputs inputs){

        }


}
