
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import frc.robot.subsystems.Breakbeam;



import frc.robot.constants;

public class Intermediate extends SubsystemBase {
    
    
    private CANSparkMax m_ForwardMotor;
    private CANSparkMax m_DirectionMotor;

    private Breakbeam fowardBreakbeam;
    private Breakbeam directionalBreakbeam;

    private final int MotorRPM = 100;

    private RelativeEncoder e_Forward; // encoder for neo's
    private RelativeEncoder e_Directional; // encoder for neo's
    

    public Intermediate(int intermediateForwardID, int intermediateDirectionID, int ledPinForward, int receiverPinFoward) {
        m_ForwardMotor = new CANSparkMax(intermediateForwardID, MotorType.kBrushless);
        m_DirectionMotor = new CANSparkMax(intermediateDirectionID, MotorType.kBrushless);

        e_Forward = m_ForwardMotor.getEncoder();
        e_Directional = m_DirectionMotor.getEncoder();

        this.resetEncoders();
    }
    
    public void goUp() {
        m_ForwardMotor.set(MotorRPM);
        m_ForwardMotor.set(-(MotorRPM));
        e_Forward.getPosition();
        e_Directional.getPosition();
        

    }
    public void goFoward() {
        m_ForwardMotor.set(-(MotorRPM));
        m_DirectionMotor.set(MotorRPM);
        e_Forward.getPosition();
        e_Directional.getPosition();

    }
    public void ejectIntermediate() {
        m_ForwardMotor.set(-(MotorRPM));
        m_DirectionMotor.set(-(MotorRPM));
        e_Forward.getPosition();
        e_Directional.getPosition();

    }

    public void resetEncoders() {
        e_Forward.setPosition(0.0);
        e_Directional.setPosition(0.0);
        
    }



}