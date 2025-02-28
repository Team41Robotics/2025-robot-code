package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AlgaeIOSparkMax implements AlgaeIO{
        
        private final DutyCycleEncoder pivotEncoder;
        private final CANSparkMax algaeMotor;
        private final double offset; //still need to tune 

        public AlgaeIOSparkMax {
                pivotEncoder = new DutyCycleEncoder(1);
                algaeMotor = new CANSparkMax(0, null);
        }
        @Override 
        public void updateInputs(AlgaeIOInputs inputs){
                inputs.algaeRotation = MathUtil.angleModulus(pivotEncoder.get() * 2 * Math.PI - offset);
                inputs.algaePivotVoltage = algaeMotor.getBusVoltage();
                inputs.algaeAngVel = algaeMotor.getEncoder().getVelocity() * 2 * Math.PI / 60.0;
                inputs.algaePivotCurrent = new double[]{algaeMotor.getOutputCurrent()};
        }
        @Override
        public void setAlgaeVoltage(double voltage){
                algaeMotor.setVoltage(MathUtil.clamp(voltage, -4, 4));
        }
}
