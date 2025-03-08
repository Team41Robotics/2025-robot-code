// package frc.robot.commands.algae;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import static frc.robot.RobotContainer.algae;

// public class ScoreAlgae extends SequentialCommandGroup{
//         public ScoreAlgae(){
//                 super(
//                         new InstantCommand(() -> algae.runIntake(true)),
//                         new WaitCommand(2),
//                         new InstantCommand(() -> algae.stopMotors())
//                 );
//                 addRequirements(algae);
//         }
// }
