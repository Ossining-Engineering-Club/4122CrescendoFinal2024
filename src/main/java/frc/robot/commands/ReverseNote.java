package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.Leds;
import frc.robot.constants.Direction;

public class ReverseNote extends Command {
    private final Intake m_intake;
    private final ShooterFeeder m_shooter;
    public final Leds m_leds;

    public ReverseNote(Intake intake, ShooterFeeder shooter, Leds led) {
        m_intake = intake;
        m_shooter = shooter;
        m_leds = led;
        addRequirements(intake, shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setReverse(true);
        m_shooter.enableFeeder();
        m_intake.setReverse(true);
        m_intake.start();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.disableFeeder();
        m_intake.stop();
        m_leds.setGreen();
        m_shooter.setReverse(false);
        m_intake.setReverse(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
