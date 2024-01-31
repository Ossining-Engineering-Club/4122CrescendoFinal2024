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
    private final CANSparkMax m_dumperPivotMotor;
    private final CANSparkMax m_dumperReleaseMotor;

    private final RelativeEncoder m_elevatorRaisingEnc;
    private final RelativeEncoder m_noteElevatorEnc;
    private final RelativeEncoder m_dumperPivotEnc;
    private final RelativeEncoder m_dumperReleaseEnc;

    private final ProfiledPIDController m_dumperPivotPID =
        new ProfiledPIDController(
            constants.kDumperPivotPIDGains[0],
            constants.kDumperPivotPIDGains[1],
            constants.kDumperPivotPIDGains[2],
            new TrapezoidProfile.Constraints(constants.kDumperPivotMaxVelocity, constants.kDumperPivotMaxAcceleration)
        );

    public Elevator(int dumperPivotMotorID, int nElevatorID, int ElevatorRaiseID, int dumperReleaseMotorID){
        m_dumperPivotMotor = new CANSparkMax(dumperPivotMotorID, MotorType.kBrushless);
        m_dumperPivotEnc = m_dumperPivotMotor.getEncoder();

        m_dumperPivotEnc.setPositionConversionFactor(2*Math.PI / constants.kDumperPivotGearing);
        m_dumperPivotEnc.setVelocityConversionFactor(2*Math.PI/60 / constants.kDumperPivotGearing);

        m_dumperPivotEnc.setPosition(0);

        m_dumperPivotPID.reset(m_dumperPivotEnc.getPosition());

        m_noteElevatorMotor = new CANSparkMax(nElevatorID, MotorType.kBrushless);
        m_noteElevatorEnc = m_noteElevatorMotor.getEncoder();

        m_dumperReleaseMotor = new CANSparkMax(dumperReleaseMotorID, MotorType.kBrushless);
        m_dumperReleaseEnc = m_dumperReleaseMotor.getEncoder();

        m_elevatorRaisingMotor = new CANSparkMax(ElevatorRaiseID, MotorType.kBrushless);
        m_elevatorRaisingEnc = m_elevatorRaisingMotor.getEncoder();
    }  

    public void ElevatorMove(double elevatorMotorSpeed){
        m_elevatorRaisingMotor.set(elevatorMotorSpeed);
        m_elevatorRaisingEnc.getPosition();
    }

    public void NoteElevator(double nElevatorMotorSpeed){
        m_noteElevatorMotor.set(nElevatorMotorSpeed);
    }

    public void DumperPivotMove(double dumperPivotMotorSpeed){
        m_dumperPivotMotor.set(dumperPivotMotorSpeed);
    }   
     
    public void DumperRelease(double dumperReleaseMotorSpeed){
        m_dumperReleaseMotor.set(dumperReleaseMotorSpeed);
    }

    @Override
    public void periodic(){
        m_dumperPivotMotor.set(m_dumperPivotPID.calculate(getAngle().getRadians()) + m_dumperPivotPID.getSetpoint().velocity);
    }

    public void setPosition(double positionRadians){
        m_dumperPivotPID.setGoal(positionRadians);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_dumperPivotEnc.getPosition());
    }
}


