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

public class Climber extends SubsystemBase {
    private final CANSparkMax m_climberMotor;
    private final RelativeEncoder e_climberMotorEnc;

    private final ProfiledPIDController m_pid = new ProfiledPIDController(
        constants.kClimberPIDGains[0],
        constants.kClimberPIDGains[1],
        constants.kClimberPIDGains[2],
        new TrapezoidProfile.Constraints(constants.kElevatorMaxSpeed, constants.kElevatorMaxAcceleration)
    );

    public Climber(int ClimberID) {
        m_climberMotor = new CANSparkMax(ClimberID, MotorType.kBrushless);
        e_climberMotorEnc = m_climberMotor.getEncoder();
    }


}