
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.constants.Direction;
import frc.robot.subsystems.Breakbeam;
import frc.robot.constants;

public class Intermediate extends SubsystemBase {
    
    private CANSparkMax m_ForwardMotor;
    private CANSparkMax m_DirectionMotor;

    private Spark ledSpark;
    private Breakbeam shooterBreakbeam;
    private Breakbeam elevatorBreakbeam;

    private double LEDVoltage;

    private Direction shooterDirection;
    private Direction elevatorDirection;

    public Intermediate(int intermediateForwardID, int intermediateDirectionID, int shooterBreakbeamPin, int elevatorBreakbeamPin, int ledPinForward, int ledPort) {
        m_ForwardMotor = new CANSparkMax(intermediateForwardID, MotorType.kBrushless);
        m_DirectionMotor = new CANSparkMax(intermediateDirectionID, MotorType.kBrushless);
        shooterDirection = Direction.STOPPED;
        elevatorDirection = Direction.STOPPED;
        ledSpark = new Spark(ledPort);
        shooterBreakbeam = new Breakbeam(shooterBreakbeamPin);
        elevatorBreakbeam = new Breakbeam(elevatorBreakbeamPin);
    }
    public void moveToElevator() {
        m_ForwardMotor.set(constants.kIntermediateMotorPower);
        m_DirectionMotor.set(-(constants.kIntermediateMotorPower));
        elevatorDirection = Direction.FORWARD;
        shooterDirection = Direction.STOPPED;
    }
    public void moveToShooter() {
        m_ForwardMotor.set(constants.kIntermediateMotorPower);
        m_DirectionMotor.set(constants.kIntermediateMotorPower);
        elevatorDirection = Direction.STOPPED;
        shooterDirection = Direction.FORWARD;
    }
    public void ejectIntermediateElevator() {
        m_ForwardMotor.set(-(constants.kIntermediateMotorPower));
        m_DirectionMotor.set(constants.kIntermediateMotorPower);
        elevatorDirection = Direction.REVERSE;
        shooterDirection = Direction.STOPPED;
    }
    public void ejectIntermediateShooter() {
        m_ForwardMotor.set(-(constants.kIntermediateMotorPower));
        m_DirectionMotor.set(-constants.kIntermediateMotorPower);
        elevatorDirection = Direction.STOPPED;
        shooterDirection = Direction.REVERSE;
    }
    public void stopIntermediate() {
        m_ForwardMotor.set(0.0);
        m_DirectionMotor.set(0.0);

        LEDVoltage = 0;
        ledSpark.setVoltage(LEDVoltage);
        elevatorDirection = Direction.STOPPED;
        shooterDirection = Direction.STOPPED;
    }
    public Direction getShooterDirection() {
        return shooterDirection;
    }
    public Direction getElevatorDirection() {
        return elevatorDirection;
    }
    public boolean ElevatorBBisTripped(){
        return elevatorBreakbeam.isTripped();
    }
    public boolean ShooterBBisTripped() {
        return shooterBreakbeam.isTripped();
    }
}