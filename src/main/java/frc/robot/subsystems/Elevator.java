package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
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

        m_noteElevatorMotor = new CANSparkMax(nElevatorID, MotorType.kBrushless);
        m_noteElevatorEnc = m_noteElevatorMotor.getEncoder();

        m_dumperReleaseMotor = new CANSparkMax(dumperReleaseMotorID, MotorType.kBrushless);
        m_dumperReleaseEnc = m_dumperReleaseMotor.getEncoder();

        m_elevatorRaisingMotor = new CANSparkMax(ElevatorRaiseID, MotorType.kBrushless);
        m_elevatorRaisingEnc = m_elevatorRaisingMotor.getEncoder();
    }  

    public void ElevatorMove(double elevatorMotorSpeed){
        m_elevatorRaisingMotor.set(elevatorMotorSpeed);
    }

    public void NoteElevatorMove(double nElevatorMotorSpeed){
        m_noteElevatorMotor.set(nElevatorMotorSpeed);
    }

    public void DumperRelease(double dumperReleaseMotorSpeed){
        m_dumperReleaseMotor.set(dumperReleaseMotorSpeed);
    }

    @Override
    public void periodic(){
       
    }

}


