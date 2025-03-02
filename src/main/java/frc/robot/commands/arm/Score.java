package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.intake;

public class Score extends Command{
        public Score(){
                addRequirements(intake);
        }
}
