package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeRun extends Command {
    private final Intake m_intake;

    public IntakeRun(Intake intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_intake.start();
    }

    public void end(boolean interrupted) {
        m_intake.stop();
    }
}
