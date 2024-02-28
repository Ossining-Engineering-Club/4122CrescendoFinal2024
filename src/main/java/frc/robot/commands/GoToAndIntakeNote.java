package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.GoToNote;
import frc.robot.subsystems.Shooter;

public class GoToAndIntakeNote extends ParallelDeadlineGroup {
    public GoToAndIntakeNote(
        Drivetrain drivetrain,
        Limelight limelight,
        Intake intake,
        Shooter shooter) {
            super(
                new IntakeNoteToShooter(intake, shooter),
                new GoToNote(drivetrain, limelight, intake)
            );
    }
}
