package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;
import frc.robot.constants.Direction;

public class ShooterFeeder extends SubsystemBase {

    private CANSparkMax m_Feeder;
    private Breakbeam m_breakbeam;

    // private final LEDController m_LedController;

    public double v_startAngle = 58.0;
    private boolean is_backward;


  public ShooterFeeder(
    int feederPort,
    int breakbeamPin){
        m_Feeder = new CANSparkMax(feederPort,MotorType.kBrushless);
        m_breakbeam = new Breakbeam(breakbeamPin);
        is_backward=false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("shooter BB", BBisTripped());
        SmartDashboard.putBoolean("feeder isReversed", is_backward);
    }

    public void enableFeeder() {
        if(this.is_backward)
            m_Feeder.set(-constants.kShooterFeederSpeed);
        else
            m_Feeder.set(constants.kShooterFeederSpeed);
    }
    public void enableAmpFeeder() {
        if(this.is_backward)
            m_Feeder.set(-constants.kShooterFeederAmpSpeed);
        else
            m_Feeder.set(constants.kShooterFeederAmpSpeed);
    }
    public void disableFeeder() {
        m_Feeder.set(0.0);
    }
    public void setReverse(boolean isOn){
        this.is_backward=isOn;
    }

    public boolean BBisTripped() {
        return m_breakbeam.isTripped();
    }

    public Direction getDirection() {
        if (m_Feeder.get() == 0.0) return Direction.STOPPED;
        if (is_backward) return Direction.REVERSE;
        return Direction.FORWARD;
    }

}
