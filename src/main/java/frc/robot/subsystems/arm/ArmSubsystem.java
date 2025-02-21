package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{

        private ArmIOHardware io;
        private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
        
        private Rotation2d shoulderTargetRotation;
        // private Rotation2d wristTargetRotation;

        public void setShoulderTargetRotation(Rotation2d target){
                this.shoulderTargetRotation = target;
        }

        public void setTargetExtension(double extension){
                
        }


        public void init(){
                io = new ArmIOHardware();
        }

        public void periodic(){
                io.updateInputs(inputs);
        }

        public double getExtension(){
                return inputs.telescopePosition;
        }

}       
