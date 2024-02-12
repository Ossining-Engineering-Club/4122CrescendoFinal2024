package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.constants;
import frc.robot.subsystems.Breakbeam;
import frc.robot.constants.Direction;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final Breakbeam m_breakbeam;
    private Direction m_direction = Direction.STOPPED;
    public Intake(int intakeMotorID, int breakbeamReceiverPin) {
        m_intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
        m_breakbeam = new Breakbeam(breakbeamReceiverPin);
    }
    public void setVelocity(double power) {
        m_intakeMotor.set(power);
    }
    public void start() {
        setVelocity(constants.kIntakePower);
        m_direction = Direction.FORWARD;
    }
    public void stop() {
        setVelocity(0);
        m_direction = Direction.STOPPED;
    }

    public void reverse() {
        setVelocity(-constants.kIntakePower);
        m_direction = Direction.REVERSE;
    }
    public boolean BBisTripped() {
        return m_breakbeam.isTripped();
    }
    public Direction getDirection() {
        return m_direction;
    }

}