package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import java.lang.Runnable;
import frc.robot.subsystems.Intermediate;
import frc.robot.constants;

public class DumperRelease extends Command {

    private final Elevator m_elevatorSubsystem;
    private final Intermediate m_intermediarySubsystem;
    private final Runnable updateState;

    public DumperRelease(Elevator elevator, Intermediate intermediate, Runnable updateState) {
        m_elevatorSubsystem = elevator;
        m_intermediarySubsystem = intermediate;
        this.updateState = updateState;
    
        addRequirements(m_elevatorSubsystem, m_intermediarySubsystem);
    }

    @Override 
    public void initialize() {
        m_elevatorSubsystem.DumperRelease(0.5); 
    }

    @Override 
    public void execute() {
      
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.DumperRelease(0.0);
        updateState.run();
    }

    @Override
    public boolean isFinished() {
        return !m_intermediarySubsystem.ElevatorBBisTripped();
    }
}