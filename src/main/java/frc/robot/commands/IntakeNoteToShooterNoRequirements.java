package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Leds;

public class IntakeNoteToShooterNoRequirements extends Command {
    private final Intake m_intake;
    private final Shooter m_shooter;
    private final Leds m_Leds;

    public IntakeNoteToShooterNoRequirements(Intake intake, Shooter shooter, Leds led) {
        m_intake = intake;
        m_shooter = shooter;
        m_Leds = led;
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
        m_Leds.setGreen();
    }

    @Override
    public boolean isFinished() {
        return m_shooter.BBisTripped();
    }
}
