package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.constants;

public class DriveAtPower extends Command {
    private final Drivetrain m_drive;
    private final double m_power;

    public DriveAtPower(Drivetrain drive, double power) {
        m_drive = drive;
        m_power = power;
    }

    @Override
    public void initialize() {
        m_drive.Drive(m_power*constants.kMaxSpeed, 0.0, 0.0, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.Drive(0.0, 0.0, 0.0, false);
    }
}
