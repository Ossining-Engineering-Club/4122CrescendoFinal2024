package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.constants;

public class ElevatorManualControl extends Command {
    private final Elevator m_elevator;
    private final DoubleSupplier m_heightSupplier;

    public ElevatorManualControl(Elevator elevator, DoubleSupplier heightSupplier) {
        m_heightSupplier = heightSupplier;
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        m_elevator.setHeight(m_elevator.getHeight()+m_heightSupplier.getAsDouble()*constants.kElevatorManualAngleControlSpeedMultiplier);
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
