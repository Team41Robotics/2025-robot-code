package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.RobotContainer.arm;

public class MicroAdjust extends Command{
        
        private double increment;

        public MicroAdjust(double _increment){
                this.increment = _increment;
                addRequirements(arm);
        }

        @Override
        public void execute(){
                Rotation2d adjustedRotation = Rotation2d.fromRadians(arm.getShoulderAngleRadians() + increment);
                arm.setShoulderTargetRotation(adjustedRotation);
        }

        @Override
        public boolean isFinished(){
                return arm.shoulderAtSetpoint();
        }

}       
