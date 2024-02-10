package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Climber;

import frc.robot.constants;

public class ClimberMove extends Command {

    private final Climber m_climberSubsystem;
    
    public ClimberMove(Climber climber){
        m_climberSubsystem = climber;

        addRequirements(m_climberSubsystem);
    }

    @Override
    public void initialize() {
    
    }
}
