package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class IntakeNoteToShooter extends Command {
    //private final Intake m_intake;
    private final Shooter m_shooter;

    public IntakeNoteToShooter(/*Intake intake, */Shooter shooter) {
        //m_intake = intake;
        m_shooter = shooter;
        //addRequirements(/*intake, */shooter);
    }

    @Override
    public void initialize() {
        m_shooter.enableFeeder();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.disableFeeder();
    }

    @Override
    public boolean isFinished() {
        return m_shooter.BBisTripped();
    }
}
