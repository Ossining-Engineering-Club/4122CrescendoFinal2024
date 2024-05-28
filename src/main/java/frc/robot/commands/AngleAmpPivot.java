package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpPivot;

public class AngleAmpPivot extends Command {
    private final AmpPivot m_ampPivot;
    private final double m_angle;

    public AngleAmpPivot(AmpPivot ampPivot, double angle) {
        m_ampPivot = ampPivot;
        m_angle = angle;

        addRequirements(m_ampPivot);
    }

    @Override
    public void execute() {
        m_ampPivot.setAngle(m_angle);
    }

    @Override
    public boolean isFinished() {
        return m_ampPivot.isAngleReached();
    }

    @Override
    public void end(boolean interrupted) {
        m_ampPivot.stopMotor();
    }
}
