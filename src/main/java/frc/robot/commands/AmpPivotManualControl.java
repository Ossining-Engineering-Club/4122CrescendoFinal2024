package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpPivot;

public class AmpPivotManualControl extends Command {
    private AmpPivot m_ampPivot;
    private DoubleSupplier m_speedSupplier;

    public AmpPivotManualControl(AmpPivot ampPivot, DoubleSupplier speedSupplier) {
        m_ampPivot = ampPivot;
        m_speedSupplier = speedSupplier;

        addRequirements(m_ampPivot);
    }

    // @Override
    // public void execute() {
    //     m_ampPivot.setMotor();
    // }
}
