package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.constants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax m_elevatorRaisingMotor;
    private final CANSparkMax m_noteElevatorMotor;
    private final CANSparkMax m_dumperReleaseMotor;

    private final RelativeEncoder m_elevatorRaisingEnc;
    private final RelativeEncoder m_noteElevatorEnc;
    private final RelativeEncoder m_dumperReleaseEnc;

    public Elevator(int nElevatorID, int ElevatorRaiseID, int dumperReleaseMotorID){
        //creating motors and creating/resetting encoders
        m_noteElevatorMotor = new CANSparkMax(nElevatorID, MotorType.kBrushless);
        m_noteElevatorEnc = m_noteElevatorMotor.getEncoder();
        m_noteElevatorEnc.setPosition(0.0);

        m_dumperReleaseMotor = new CANSparkMax(dumperReleaseMotorID, MotorType.kBrushless);
        m_dumperReleaseEnc = m_dumperReleaseMotor.getEncoder();
        m_dumperReleaseEnc.setPosition(0.0);

        m_elevatorRaisingMotor = new CANSparkMax(ElevatorRaiseID, MotorType.kBrushless);
        m_elevatorRaisingEnc = m_elevatorRaisingMotor.getEncoder();
        m_elevatorRaisingEnc.setPosition(0.0);
    }  

    //Resetting encoders
    public void resetElevatorEncoder(){
        m_elevatorRaisingEnc.setPosition(0.0);
    }

    public void resetNElevatorEncoder(){
        m_noteElevatorEnc.setPosition(0.0);
    }

    public void resetDumperEncoders(){
        m_dumperReleaseEnc.setPosition(0.0);
    }
    
    //Setting motor speeds
    public void ElevatorMove(double elevatorMotorSpeed){
        m_elevatorRaisingMotor.set(elevatorMotorSpeed);
    }

    public void NoteElevatorMove(double nElevatorMotorSpeed){
        m_noteElevatorMotor.set(nElevatorMotorSpeed);
    }

    public void DumperRelease(double dumperReleaseMotorSpeed){
        m_dumperReleaseMotor.set(dumperReleaseMotorSpeed);
    }

    //Getting encoder values
    public double getElevatorEncoder(){
        return m_elevatorRaisingEnc.getPosition();
    }

    public double getNElevatorEncoder(){
        return m_noteElevatorEnc.getPosition();
    }

    public double getDumperEncoder(){
        return m_dumperReleaseEnc.getPosition();
    }

    @Override
    public void periodic(){
       
    }

}


