package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Elevator;
import frc.robot.constants;

public class ElevatorExtend extends Command {
    private final Elevator m_elevatorSubsystem;

    private final ProfiledPIDController m_pid = new ProfiledPIDController(
        constants.kElevatorPIDGains[0],
        constants.kElevatorPIDGains[1],
        constants.kElevatorPIDGains[2],
        new TrapezoidProfile.Constraints(constants.kElevatorMaxSpeed, constants.kElevatorMaxAcceleration)
    );

    private final double m_goal;

    public ElevatorExtend(Elevator elevator, double goal) {
        m_elevatorSubsystem = elevator;
        m_goal = goal;

        addRequirements(m_elevatorSubsystem);
    }

    @Override 
    public void initialize() {
        m_pid.reset(m_elevatorSubsystem.getElevatorEncoder());
        m_pid.setGoal(m_goal);
    }

    @Override 
    public void execute() {
        m_elevatorSubsystem.ElevatorMove(m_pid.calculate(m_elevatorSubsystem.getElevatorEncoder())+m_pid.getSetpoint().velocity);
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSubsystem.ElevatorMove(0.0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_elevatorSubsystem.getElevatorEncoder()-m_goal) < 0.01;
    }
}