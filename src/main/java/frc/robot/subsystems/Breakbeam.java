package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Breakbeam extends SubsystemBase {
    //private final DigitalInput m_receiver;
    private final AnalogInput m_receiver;

    public Breakbeam(int receiverPin) {
        //m_receiver = new DigitalInput(receiverPin);
        m_receiver = new AnalogInput(receiverPin);
    }

    public boolean isTripped() {
        //return m_receiver.get();
        return m_receiver.getVoltage() > 3.0;
    }

    

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("breakbeam is tripped?", isTripped());
        SmartDashboard.putNumber("breakbeam voltage", m_receiver.getVoltage());
    }
}