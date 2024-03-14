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

public class ShooterFlywheels extends SubsystemBase {

    public CANSparkFlex m_Shooter1;
    public CANSparkFlex m_Shooter2;
    private RelativeEncoder e_Shooter1;
    private RelativeEncoder e_Shooter2;
    //No KA because very little
    // public SimpleMotorFeedforward Shooter1PIDController = new SimpleMotorFeedforward(kS, kV);
    // public SimpleMotorFeedforward Shooter2PIDController = new SimpleMotorFeedforward(kS, kV);
    public PIDController Shooter1PIDController = new PIDController(constants.kShooter1PIDGains[0],
                                                                   constants.kShooter1PIDGains[1],
                                                                   constants.kShooter1PIDGains[2]);
    public PIDController Shooter2PIDController = new PIDController(constants.kShooter2PIDGains[0],
                                                                   constants.kShooter2PIDGains[1],
                                                                   constants.kShooter2PIDGains[2]);


  public ShooterFlywheels(
    int Flywheelport1,
    int Flywheelport2){
        m_Shooter1 = new CANSparkFlex(Flywheelport1,MotorType.kBrushless);
        m_Shooter2 = new CANSparkFlex(Flywheelport2,MotorType.kBrushless);
        e_Shooter1 = m_Shooter1.getEncoder();
        e_Shooter2 = m_Shooter2.getEncoder();
        
        e_Shooter1.setVelocityConversionFactor(constants.kShooterGearRatio);
        e_Shooter2.setVelocityConversionFactor(constants.kShooterGearRatio);
        
        this.resetEncoders();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("shooter RPM", e_Shooter2.getVelocity());
    }

    public void resetEncoders(){
        e_Shooter1.setPosition(0.0);
        e_Shooter2.setPosition(0.0);
    }
    //sets shooter flywheel voltage
    public void setFlywheelsVoltage(double setPoint){
        m_Shooter1.setVoltage(setPoint);
        m_Shooter2.setVoltage(setPoint);
    }

    //returns true if setpoint is reached false otherwise
    public boolean setRPM(double targetRPM){
        double currentRPMS1 = e_Shooter1.getVelocity();
        double currentRPMS2 = e_Shooter2.getVelocity();

        Shooter1PIDController.setSetpoint(targetRPM);
        Shooter2PIDController.setSetpoint(targetRPM);

        double adjustmentvalS1 = Shooter1PIDController.calculate(currentRPMS1);
        double adjustmentvalS2 = Shooter2PIDController.calculate(currentRPMS2);

        m_Shooter1.set(m_Shooter1.get()+adjustmentvalS1);
        m_Shooter2.set(m_Shooter2.get()+adjustmentvalS2);

        // SmartDashboard.putNumber("Shooter Setpoint", targetRPM);
        // SmartDashboard.putNumber("Shooter RPM", e_Shooter1.getVelocity());
        // SmartDashboard.putNumber("Shooter Angle", getAngle());

        // m_Shooter1.set(targetRPM/6000);
        // m_Shooter2.set(targetRPM/6000);

        if(Math.abs(Shooter1PIDController.getSetpoint()-currentRPMS1) < constants.kRPMTolerance && 
            Math.abs(Shooter2PIDController.getSetpoint()-currentRPMS2) < constants.kRPMTolerance){
            return true;
        }else{
            return false;
        }
    }

    public void stopFlywheels() {
        m_Shooter1.set(0);
        m_Shooter2.set(0);
    }

    public boolean isRPMReached() {
        double targetRPM = Shooter1PIDController.getSetpoint();
        return Math.abs(Shooter1PIDController.getSetpoint()-e_Shooter1.getVelocity()) < constants.kRPMTolerance && 
                Math.abs(Shooter2PIDController.getSetpoint()-e_Shooter2.getVelocity()) < constants.kRPMTolerance;
    }
}
