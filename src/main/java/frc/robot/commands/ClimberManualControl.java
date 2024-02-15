package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Climber;

import frc.robot.constants;

public class ClimberManualControl extends Command {
    private final Climber m_climber;
    private final DoubleSupplier m_heightSupplier;

    private final ProfiledPIDController m_pid = new ProfiledPIDController(
        constants.kClimberPIDGains[0],
        constants.kClimberPIDGains[1],
        constants.kClimberPIDGains[2],
        new TrapezoidProfile.Constraints(constants.kClimberMaxSpeed, constants.kClimberMaxAcceleration)
    );
    
    public ClimberManualControl(Climber climber, DoubleSupplier heightSupplier){
        m_climber = climber;
        m_heightSupplier = heightSupplier;

        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        m_pid.reset(m_climber.getPosition());
    }

    @Override
    public void execute() {
        m_pid.setGoal(m_heightSupplier.getAsDouble());
        m_climber.setSpeed(m_pid.calculate(m_climber.getPosition())+m_pid.getSetpoint().velocity);
    }
    
    @Override 
    public void end(boolean interrupted) {
        m_climber.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
