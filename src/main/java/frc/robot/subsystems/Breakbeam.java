package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants;

public class Breakbeam extends SubsystemBase {
    private final AnalogInput m_receiver;
    private int m_onCount = 0;

    public Breakbeam(int receiverPin) {
        m_receiver = new AnalogInput(receiverPin);
    }

    public boolean isTripped() {
        if (m_receiver.getVoltage() < constants.k_BreakbeamVoltageThreshold) m_onCount++;
        else m_onCount = 0;
        return m_onCount >= constants.k_BreakbeamSamplingWindow;
    }

    public double getVoltage() {
        return m_receiver.getVoltage();
    }

    @Override
    public void periodic() {
        
    }
}