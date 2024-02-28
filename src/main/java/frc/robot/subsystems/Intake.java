package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.constants;
import frc.robot.subsystems.Breakbeam;
import frc.robot.constants.Direction;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotorTop;
    private final CANSparkMax m_intakeMotorBottom;
    // /private final Breakbeam m_breakbeam;
    private Direction m_direction = Direction.STOPPED;
    public Intake(int intakeMotorTopID, int intakeMotorBottomID/*, int breakbeamReceiverPin*/) {
        m_intakeMotorTop = new CANSparkMax(intakeMotorTopID, MotorType.kBrushless);
        m_intakeMotorBottom = new CANSparkMax(intakeMotorBottomID, MotorType.kBrushless);
        //m_breakbeam = new Breakbeam(breakbeamReceiverPin);
    }
    public void setVelocity(double power) {
        m_intakeMotorTop.set(power);
        m_intakeMotorBottom.set(power);
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
        //return m_breakbeam.isTripped();
        return false;
    }
    public Direction getDirection() {
        return m_direction;
    }

}