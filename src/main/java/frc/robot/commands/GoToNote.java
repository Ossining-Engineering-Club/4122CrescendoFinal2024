package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class GoToNote extends Command {
    private final Drivetrain m_drive;
    private final Limelight m_limelight;

    private final ProfiledPIDController m_transPIDController = new ProfiledPIDController(
        constants.kVisionTransPIDGains[0],
        constants.kVisionTransPIDGains[1],
        constants.kVisionTransPIDGains[2],
        new TrapezoidProfile.Constraints(constants.kMaxSpeed, constants.kMaxAcceleration)
    );

    private final ProfiledPIDController m_rotPIDController = new ProfiledPIDController(
        constants.kVisionRotPIDGains[0],
        constants.kVisionRotPIDGains[1],
        constants.kVisionRotPIDGains[2],
        new TrapezoidProfile.Constraints(constants.kMaxAngularSpeed, constants.kMaxAngularAcceleration)
    );

    private double prevTX;
    private double prevTransAngle;

    public GoToNote(Drivetrain drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_transPIDController.reset(Math.sqrt(m_limelight.getTA()/100));
        m_rotPIDController.reset(-m_limelight.getTX()/180*Math.PI);
        m_transPIDController.setGoal(1);
        m_rotPIDController.setGoal(m_drive.getAngle().getRadians()-m_limelight.getTX()/180*Math.PI);
        prevTX = -m_limelight.getTX()/180*Math.PI;
        prevTransAngle = m_drive.getAngle().getRadians()-m_limelight.getTX()/180*Math.PI;
    }

    @Override
    public void execute() {
        if (m_limelight.hasTarget()) {
            double tx = -m_limelight.getTX()/180*Math.PI;
            double ta = m_limelight.getTA()/100;

            if (Math.abs(tx) < constants.kVisionXToleranceRadians) tx = 0;

            double overallSpeed = 1.0/*m_transPIDController.calculate(Math.sqrt(ta))*/;
            double transAngle = m_drive.getAngle().getRadians()+tx;
            // if the limelight measurement is old, use the previous transAngle
            if (Math.abs(tx-prevTX) < 0.00001) transAngle = prevTransAngle;
            else {
                prevTX = tx;
                prevTransAngle = transAngle;
            }
            double xSpeed = overallSpeed*Math.cos(transAngle);
            double ySpeed = overallSpeed*Math.sin(transAngle);

            double rotSpeed = MathUtil.clamp(
                -m_rotPIDController.calculate(-m_drive.getAngle().getRadians())-m_rotPIDController.getSetpoint().velocity,
                -constants.kMaxAngularSpeed,
                constants.kMaxAngularSpeed);

            SmartDashboard.putNumber("tx", tx);
            //SmartDashboard.putNumber("sqrt(ta)", Math.sqrt(ta));
            SmartDashboard.putNumber("transAngle", transAngle);
            
            m_drive.Drive(xSpeed, ySpeed, rotSpeed, true);
        }
        else {
            m_drive.Drive(0, 0, 0, true);
        }

        //SmartDashboard.putNumber("vision x pid setpoint", m_xPIDController.getSetpoint().position);
    }

    @Override
    public boolean isFinished() {
        return !m_limelight.hasTarget();
    }

}