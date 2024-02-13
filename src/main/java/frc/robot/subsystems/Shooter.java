package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class Shooter extends SubsystemBase{


    private CANSparkMax m_Shooter1;
    private CANSparkMax m_Shooter2;
    private CANSparkMax m_Angle;
    private RelativeEncoder e_Angle;
    private RelativeEncoder e_Shooter1;
    private RelativeEncoder e_Shooter2;
    public PIDController Shooter1PIDController = new PIDController(constants.kShooter1PIDGains[0],
                                                                   constants.kShooter1PIDGains[1],
                                                                   constants.kShooter1PIDGains[2]);
    public PIDController Shooter2PIDController = new PIDController(constants.kShooter2PIDGains[0],
                                                                   constants.kShooter2PIDGains[1],
                                                                   constants.kShooter2PIDGains[2]);
    public PIDController AnglePIDController = new PIDController(constants.kAnglePIDGains[0],
                                                                constants.kAnglePIDGains[1],
                                                                constants.kAnglePIDGains[2]);


  public Shooter(int Flywheelport1,int Flywheelport2, int motorAnglePort, double startangle){
        m_Shooter1 = new CANSparkMax(Flywheelport1,MotorType.kBrushless);
        m_Shooter2 = new CANSparkMax(Flywheelport2,MotorType.kBrushless);
        m_Angle = new CANSparkMax(motorAnglePort,MotorType.kBrushless);

        e_Angle = m_Angle.getEncoder();
        e_Shooter1 = m_Shooter1.getEncoder();
        e_Shooter2 = m_Shooter2.getEncoder();
        
        e_Angle.setPositionConversionFactor(constants.kAngleRatio * constants.k_PI * 2);
        e_Shooter1.setVelocityConversionFactor(constants.kShooterGearRatio);
        e_Shooter2.setVelocityConversionFactor(constants.kShooterGearRatio);
        

        this.resetEncoders(startangle);
    }
    public void resetEncoders(double startangle){
        e_Angle.setPosition(startangle);
        e_Shooter1.setPosition(0.0);
        e_Shooter2.setPosition(0.0);
    }
    //returns true if setpoint is reached false otherwise
    public boolean setRPM(double targetRPM){
        double currentRPMS1 = e_Shooter1.getVelocity();
        double currentRPMS2 = e_Shooter2.getVelocity();

        Shooter1PIDController.setSetpoint(targetRPM);
        Shooter2PIDController.setSetpoint(targetRPM);

        double adjustmentvalS1 = Shooter1PIDController.calculate(currentRPMS1);
        double adjustmentvalS2 = Shooter2PIDController.calculate(currentRPMS2);

        m_Shooter1.set(adjustmentvalS1);
        m_Shooter1.set(adjustmentvalS2);

        if(Math.abs(targetRPM-currentRPMS1) < constants.kRPMTolerance && 
            Math.abs(targetRPM-currentRPMS2) < constants.kRPMTolerance){
            return true;
        }else{
            return false;
        }

    }
    
    //returns true if setpoint is reached false otherwise
    public boolean setAngle(double Angle) {
        AnglePIDController.setSetpoint(Angle);
        return isAngleReached();
    }

    //returns true if setpoint is reached false otherwise
    public boolean isAngleReached() {
        return Math.abs(AnglePIDController.getSetpoint()-e_Angle.getPosition()) < constants.kangleTolerance;
    }

    @Override
    public void periodic() {
        m_Angle.set(AnglePIDController.calculate(e_Angle.getPosition()));
    }
}
