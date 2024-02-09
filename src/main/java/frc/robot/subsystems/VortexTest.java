package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class VortexTest extends SubsystemBase {
    private final CANSparkFlex m_motor;

    public VortexTest(int canid) {
        m_motor = new CANSparkFlex(canid, MotorType.kBrushless);
    }

    public void drive(double speed) {
        m_motor.set(speed);
    }
}
