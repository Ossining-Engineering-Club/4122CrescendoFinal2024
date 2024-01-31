package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Breakbeam extends SubsystemBase {
    private final DigitalInput m_receiver;

    public Breakbeam(int receiverPin) {
        m_receiver = new DigitalInput(receiverPin);
    }

    public boolean isTripped() {
        return m_receiver.get();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("breakbeam is tripped?", isTripped());
    }
}