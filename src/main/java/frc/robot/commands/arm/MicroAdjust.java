package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.arm;
import frc.robot.subsystems.arm.ArmJoint;

public class MicroAdjust extends Command{
        
        private final double increment;
        private final ArmJoint joint;

        public MicroAdjust(double _increment, ArmJoint _joint){
                this.increment = _increment;
                this.joint = _joint;
                addRequirements(arm);
        }

        @Override
        public void execute(){
                switch(joint){
                        case SHOULDER -> {
                            Rotation2d adjusted = Rotation2d.fromRadians(arm.clampShoulderTargetAngle(arm.getShoulderAngleRadians() + increment));
                            arm.setShoulderTargetRotation(adjusted);
                            return;
                }
                        case EXTENSION -> {
                            double adjusted = arm.clampTargetExtension(arm.getExtension() + increment);
                            arm.setTargetExtension(adjusted);
                            return;
                }
                        case DIH -> {
                            double adjusted = arm.clampWristTargetAngle(arm.getWristAngle() + increment);
                            arm.setWristTargetRotation(adjusted);
                            return;

                }

                }
        }

        @Override
        public boolean isFinished(){
                switch(joint){
                        case SHOULDER -> {
                                return arm.shoulderAtSetpoint();
                        }
                        case EXTENSION -> {
                                return arm.extensionAtSetpoint();
                        }
                        case DIH -> {
                                return arm.wristAtSetpoint();
                        }
                        default -> {
                                return false;
                        }
                }
        }

}       
