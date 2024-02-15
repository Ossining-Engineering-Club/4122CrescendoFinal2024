package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.constants;

public class SearchForNote extends Command {
    private final Drivetrain m_drive;
    private final Limelight m_limelight;
    private final double m_xLimit;
    private final boolean m_isGoingPositive;

    public SearchForNote(Drivetrain drive, Limelight limelight, double xLimit, boolean isGoingPositive) {
        m_drive = drive;
        m_limelight = limelight;
        m_xLimit = xLimit;
        m_isGoingPositive = isGoingPositive;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        if (m_isGoingPositive) m_drive.Drive(constants.kNoteSearchingSpeed, 0.0, 0.0, true);
        else m_drive.Drive(-constants.kNoteSearchingSpeed, 0.0, 0.0, true);
    }

    @Override
    public boolean isFinished() {
        if (m_limelight.hasTarget()) return true;
        if (m_isGoingPositive) return m_drive.SwerveOdometryGetPose().getX() >= m_xLimit;
        else return m_drive.SwerveOdometryGetPose().getX() <= m_xLimit;
    }
}
