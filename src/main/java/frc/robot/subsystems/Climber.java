package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.constants;

public class Climber extends SubsystemBase {
    private final CANSparkMax m_climberMotor;
    private final RelativeEncoder e_climberMotorEnc;

    public Climber(int ClimberID) {
        m_climberMotor = new CANSparkMax(ClimberID, MotorType.kBrushless);
        e_climberMotorEnc = m_climberMotor.getEncoder();
        e_climberMotorEnc.setPositionConversionFactor(1.0 / constants.kClimberGearing);
    }

    public void setSpeed(double climberSpeed) {
        m_climberMotor.set(climberSpeed);
    }

    public double getPosition() {
        return e_climberMotorEnc.getPosition();
    }

}