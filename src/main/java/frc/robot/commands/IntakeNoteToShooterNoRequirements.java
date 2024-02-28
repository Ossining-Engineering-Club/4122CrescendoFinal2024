package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNoteToShooterNoRequirements extends Command {
    private final Intake m_intake;
    private final Shooter m_shooter;

    public IntakeNoteToShooterNoRequirements(Intake intake, Shooter shooter) {
        m_intake = intake;
        m_shooter = shooter;
    }

    @Override
    public void initialize() {
        m_shooter.enableFeeder();
        m_intake.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.disableFeeder();
        m_intake.stop();
    }

    @Override
    public boolean isFinished() {
        return m_shooter.BBisTripped();
    }
}
