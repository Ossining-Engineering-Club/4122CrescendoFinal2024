package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class GoToNoteAuto extends Command {
    private final Drivetrain m_drive;
    private final Intake m_intake;

    public GoToNoteAuto(Drivetrain drive, Intake intake) {
        m_drive = drive;
        m_intake = intake;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        m_drive.Drive(1, 0, 0, false, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.Drive(0, 0, 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return m_intake.BBisTripped();
    }
}
