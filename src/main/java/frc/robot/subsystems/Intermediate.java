
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

 
import frc.robot.subsystems.Breakbeam;



import frc.robot.constants;

public class Intermediate extends SubsystemBase {
    
    
    private CANSparkMax m_ForwardMotor;
    private CANSparkMax m_DirectionMotor;

    private Spark ledSpark;
    private Breakbeam fowardBreakbeam;
    private Breakbeam directionalBreakbeam;

    private final int MotorPower = 100;

    private double LEDVoltage;

    private RelativeEncoder e_Forward; // encoder for neo's
    private RelativeEncoder e_Directional; // encoder for neo's
    

    public Intermediate(int intermediateForwardID, int intermediateDirectionID, int ledPinForward, int receiverPinFoward, int ledPort) {
        m_ForwardMotor = new CANSparkMax(intermediateForwardID, MotorType.kBrushless);
        m_DirectionMotor = new CANSparkMax(intermediateDirectionID, MotorType.kBrushless);

        ledSpark = new Spark(ledPort);

        e_Forward = m_ForwardMotor.getEncoder();
        e_Directional = m_DirectionMotor.getEncoder();

        this.resetEncoders();
    }
    public void moveToElevator() {
        m_ForwardMotor.set(MotorPower);
        m_DirectionMotor.set(-(MotorPower));
    }
    public void moveToShooter() {
        m_ForwardMotor.set(MotorPower);
        m_DirectionMotor.set(MotorPower);
    }
    public void ejectIntermediateElevator() {
        m_ForwardMotor.set(-(MotorPower));
        m_DirectionMotor.set(MotorPower);
    }
    public void ejectIntermediateShooter() {
        m_ForwardMotor.set(-(MotorPower));
        m_DirectionMotor.set(-MotorPower);
    }
    public void stopIntermediate() {
        m_ForwardMotor.set(0.0);
        m_DirectionMotor.set(0.0);

        LEDVoltage = 0;
        ledSpark.setVoltage(LEDVoltage);
    }
    public void leftIntermediate() {
        if (fowardBreakbeam.isTripped() == true || directionalBreakbeam.isTripped() == true){
            this.stopIntermediate();
        }
    }
    public void resetEncoders() {
        e_Forward.setPosition(0.0);
        e_Directional.setPosition(0.0);
        
    }


}