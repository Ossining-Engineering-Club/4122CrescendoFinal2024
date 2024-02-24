package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.GoToNote;
import frc.robot.subsystems.Shooter;

public class GoToAndIntakeNote extends ParallelCommandGroup {
    public GoToAndIntakeNote(
        Drivetrain drivetrain,
        Limelight limelight,
        Intake intake,
        Shooter shooter) {
            addCommands(
                new IntakeNoteToShooter(shooter/*, intake*/),
                new GoToNote(drivetrain, limelight, intake)
            );
    }
}
