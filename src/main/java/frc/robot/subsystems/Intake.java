package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.constants;
import frc.robot.subsystems.Breakbeam;
import frc.robot.constants.Direction;
import frc.robot.constants.NoteState;
import edu.wpi.first.wpilibj.Timer;

public class Intake extends SubsystemBase {
    // private final CANSparkMax m_intakeMotorTop;
    // private final CANSparkMax m_intakeMotorBottom;
    private final CANSparkMax m_intakeMotor;
    private final Breakbeam m_breakbeam;
    private Direction m_direction = Direction.STOPPED;
    private boolean m_isReversed = false;

    private final LEDController m_LedController;
    private NoteState m_noteState = NoteState.EMPTY;
    private boolean m_lastIntakeBreakbeam = false;
    private boolean m_lastShooterBreeakBeam = false;
    private double transferStartTime = -1;

    public Intake(int intakeMotorID, int breakbeamReceiverPin, LEDController m_LedController) {
        // m_intakeMotorTop = new CANSparkMax(intakeMotorTopID, MotorType.kBrushless);
        // m_intakeMotorBottom = new CANSparkMax(intakeMotorBottomID, MotorType.kBrushless);
        m_intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
        m_breakbeam = new Breakbeam(breakbeamReceiverPin);

        this.m_LedController = m_LedController;
    }
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("intake beambreak voltage", m_breakbeam.getVoltage());
        SmartDashboard.putBoolean("intake BB", m_breakbeam.isTripped());

        updateNoteState();
        SmartDashboard.putString("Note State", m_noteState.toString());
        SmartDashboard.putNumber("Timeout timer", transferStartTime);
    }
    public void setVelocity(double power) {
        // m_intakeMotorTop.set(power);
        // m_intakeMotorBottom.set(power);
        m_intakeMotor.set(power);
    }
    public void start() {
        if (!m_isReversed) {
            setVelocity(-constants.kIntakePower);
            m_direction = Direction.FORWARD;
        }
        else {
            setVelocity(constants.kIntakePower);
            m_direction = Direction.REVERSE;
        }
    }
    public void stop() {
        setVelocity(0);
        m_direction = Direction.STOPPED;
    }

    public void setReverse(boolean isOn) {
        m_isReversed = isOn;
    }

    /*public void reverse() {
        setVelocity(-constants.kIntakePower);
        m_direction = Direction.REVERSE;
    }*/
    public boolean BBisTripped() {
        return m_breakbeam.isTripped();
    }
    public Direction getDirection() {
        return m_direction;
    }

    public void setShooterBBState(boolean state) {
        m_lastShooterBreeakBeam = state;
    }

    public void updateNoteState() {
        switch (m_noteState) {
            case EMPTY:
                if (BBisTripped()) {
                    m_noteState = NoteState.INTAKING;
                }
                break;

            case INTAKING:
                if (!BBisTripped()) {
                    transferStartTime = Timer.getFPGATimestamp(); // Start intermediate timer
                    m_noteState = NoteState.INTERMEDIATE;
                }
                break;

            case INTERMEDIATE:
                if (m_lastShooterBreeakBeam) {
                    m_noteState = NoteState.READY_TO_SHOOT; // Note reached the shooter
                } else if (Timer.getFPGATimestamp() - transferStartTime > 5.0) { 
                    m_noteState = NoteState.EMPTY; // Intermediate failed or timed out
                                                    // (bc lets say after 5 seconds the note hasn't made it to the shooter but it should've already )
                }
                break;

            case READY_TO_SHOOT:
                if (!m_lastShooterBreeakBeam) {
                    m_noteState = NoteState.EMPTY; // Note has left shooter
                }
                break;
        }
        m_LedController.setState(m_noteState);
    }

    
}