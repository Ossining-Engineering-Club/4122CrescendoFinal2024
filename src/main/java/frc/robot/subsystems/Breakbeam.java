package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants;

public class Breakbeam extends SubsystemBase {
    private final AnalogInput m_receiver;

    public Breakbeam(int receiverPin) {
        m_receiver = new AnalogInput(receiverPin);
    }

    public boolean isTripped() {
        return m_receiver.getVoltage() < constants.k_BreakbeamVoltageThreshold;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("beambreak voltage", m_receiver.getVoltage());
    }
}