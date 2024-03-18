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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants;
import frc.robot.constants.Direction;

import frc.robot.subsystems.Leds;

public class ShooterFlywheels extends SubsystemBase {

    public CANSparkFlex m_Shooter1;
    public CANSparkFlex m_Shooter2;
    private RelativeEncoder e_Shooter1;
    private RelativeEncoder e_Shooter2;

    private Timer m_timer = new Timer();
    private Leds m_led;

    public ShooterFlywheels(
        int Flywheelport1,
        int Flywheelport2,
        Leds led){
            m_Shooter1 = new CANSparkFlex(Flywheelport1,MotorType.kBrushless);
            m_Shooter2 = new CANSparkFlex(Flywheelport2,MotorType.kBrushless);
            e_Shooter1 = m_Shooter1.getEncoder();
            e_Shooter2 = m_Shooter2.getEncoder();

            m_led = led;
            
            e_Shooter1.setVelocityConversionFactor(constants.kShooterGearRatio);
            e_Shooter2.setVelocityConversionFactor(constants.kShooterGearRatio);
            
            this.resetEncoders();
            m_timer.reset();
            m_timer.stop();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("shooter RPM", e_Shooter2.getVelocity());
        SmartDashboard.putNumber("timer", m_timer.get());
    }

    public void resetEncoders(){
        e_Shooter1.setPosition(0.0);
        e_Shooter2.setPosition(0.0);
    }

    //sets shooter flywheel voltage
    public void setFlywheelsVoltage(double voltage){
        m_Shooter1.setVoltage(voltage);
        m_Shooter2.setVoltage(voltage);
    }

    public void start() {
        m_timer.start();
        setFlywheelsVoltage(constants.kShooterSpeakerVoltage);
         m_led.strobeWhite();
    }

    public void stop() {
        m_timer.stop();
        m_timer.reset();
        setFlywheelsVoltage(0);
    }

    public boolean isSpunUp() {
        return m_timer.get() >= constants.kShooterFlywheelSpinUpTime;
    }

    public void resetTimer() {
        m_timer.reset();
    }

    public void stopTimer() {
        m_timer.stop();
    }

    public void startTimer() {
        m_timer.start();
    }
}
