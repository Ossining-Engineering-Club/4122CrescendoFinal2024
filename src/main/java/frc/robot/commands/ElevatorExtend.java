package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorExtend extends Command {
    private final Elevator m_elevatorSubsystem;

    public ElevatorExtend(Elevator elevator) {
        m_elevatorSubsystem = elevator;

        addRequirements(m_elevatorSubsystem);
    }

    @Override 
    public void initialize() {

    }

    @Override 
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}