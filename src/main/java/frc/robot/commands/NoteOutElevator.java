package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intermediate;
import frc.robot.constants;

public class NoteOutElevator extends Command {

    private final Elevator m_elevatorSubsystem;
    private final Intermediate m_intermediarySubsystem;

    public NoteOutElevator(Elevator elevator, Intermediate intermediate) {
        m_elevatorSubsystem = elevator;
        m_intermediarySubsystem = intermediate;
    
        addRequirements(m_elevatorSubsystem);
    }

    @Override 
    public void initialize() {
        m_elevatorSubsystem.NoteElevatorMove(0.5);
    }

    @Override 
    public void execute() {
      
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.NoteElevatorMove(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}