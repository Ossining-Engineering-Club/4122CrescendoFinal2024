package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.fasterxml.jackson.databind.deser.SettableAnyProperty;
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

    private PIDController m_pid1 = new PIDController(constants.kShooter1PIDGains[0],
                                                    constants.kShooter1PIDGains[1],
                                                    constants.kShooter1PIDGains[2]);

    private PIDController m_pid2 = new PIDController(constants.kShooter2PIDGains[0],
                                                    constants.kShooter2PIDGains[1],
                                                    constants.kShooter2PIDGains[2]);

    // private SimpleMotorFeedforward m_feedforward1 = new SimpleMotorFeedforward(0, 0);
    // private SimpleMotorFeedforward m_feedforward2 = new SimpleMotorFeedforward(0, 0); // guess: kv=0.0015

    private Timer m_timer = new Timer();
    private Leds m_led;

    private double prevRPM1 = 0.0;
    private double prevRPM2 = 0.0;
    private double vTop = 8.0;
    private double vBottom = 8.0;
    private double integralTop = 0.0;
    private double integralBottom = 0.0;
    private double setpoint = 0.0;

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
        SmartDashboard.putNumber("shooter RPM 1", e_Shooter1.getVelocity());
        SmartDashboard.putNumber("shooter RPM 2", e_Shooter2.getVelocity());
        SmartDashboard.putNumber("timer", m_timer.get());
        SmartDashboard.putNumber("flywheel setpoint", setpoint);

        if (setpoint != 0.0) {
            // integralTop += setpoint-e_Shooter1.getVelocity();
            // integralBottom += setpoint-e_Shooter2.getVelocity();
            // m_Shooter2.setVoltage(vBottom + constants.kShooter2PIDGains[0]*(setpoint-e_Shooter2.getVelocity())
            //                             + constants.kShooter2PIDGains[1]*integralTop
            //                             + constants.kShooter2PIDGains[2]*(e_Shooter2.getVelocity()-prevRPM2));
            // m_Shooter1.setVoltage(vTop + constants.kShooter1PIDGains[0]*(setpoint-e_Shooter1.getVelocity())
            //                             + constants.kShooter1PIDGains[1]*integralBottom
            //                             + constants.kShooter1PIDGains[2]*(e_Shooter1.getVelocity()-prevRPM1));
            // prevRPM1 = e_Shooter1.getVelocity();
            // prevRPM2 = e_Shooter2.getVelocity();
            m_Shooter1.setVoltage(m_pid1.calculate(e_Shooter1.getVelocity(), setpoint)
                                + constants.kShooter1StartingVoltage);
            m_Shooter2.setVoltage(m_pid2.calculate(e_Shooter2.getVelocity(), setpoint)
                                + constants.kShooter2StartingVoltage);
        }
        
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

    public void setRPM(double RPM) {
        setpoint = RPM;
    }

    public void start() {
        m_timer.start();
        //setFlywheelsVoltage(constants.kShooterSpeakerVoltage);
        setRPM(constants.kShooterDefaultRPM);
        integralBottom = 0.0;
        integralTop = 0.0;
        m_led.strobeWhite();
    }

    public void stop() {
        m_timer.stop();
        m_timer.reset();
        setpoint = 0.0;
        setFlywheelsVoltage(0);
    }

    public boolean isSpunUp() {
        //return m_timer.get() >= constants.kShooterFlywheelSpinUpTime;
        return Math.abs(e_Shooter1.getVelocity()-setpoint) < 300.0 && Math.abs(e_Shooter2.getVelocity()-setpoint) < 300.0;
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
