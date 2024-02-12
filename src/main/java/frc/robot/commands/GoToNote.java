package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
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

    private final LinearFilter m_rotFilter = LinearFilter.movingAverage(50);

    public GoToNote(Drivetrain drive, Limelight limelight) {
        m_drive = drive;
        m_limelight = limelight;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_transPIDController.reset(Math.sqrt(m_limelight.getTA()/100));
        m_rotPIDController.reset(m_drive.getAngle().getRadians());
        m_transPIDController.setGoal(1);
        m_rotFilter.reset();
        for (int i = 0; i < 50; i++) m_rotFilter.calculate(m_drive.getAngle().getRadians());
        prevTX = -m_limelight.getTX()/180*Math.PI;
        prevTransAngle = m_drive.getAngle().getRadians()-m_limelight.getTX()/180*Math.PI;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("GoToNote gyro angle", m_drive.getAngle().getRadians());

        double transAngle;
        double overallSpeed = 1.0/*m_transPIDController.calculate(Math.sqrt(ta))*/;

        if (m_limelight.hasTarget()) {
            double tx = -m_limelight.getTX()/180*Math.PI;
            double ta = m_limelight.getTA()/100;

            if (Math.abs(tx) < constants.kVisionXToleranceRadians) tx = 0;

            transAngle = m_drive.getAngle().getRadians()+tx;
            // if the limelight measurement is old, use the previous transAngle
            if (Math.abs(tx-prevTX) < 0.00001) transAngle = prevTransAngle;
            else {
                prevTX = tx;
                prevTransAngle = transAngle;
            }
            SmartDashboard.putNumber("tx", tx);
        }
        else {
            transAngle = prevTransAngle;
            SmartDashboard.putNumber("tx", -9999999);
        }
        
        double xSpeed = overallSpeed*Math.cos(transAngle);
        double ySpeed = overallSpeed*Math.sin(transAngle);

        m_rotPIDController.setGoal(m_rotFilter.calculate(transAngle));
        //SmartDashboard.putNumber("GoToNote rot setpoint", m_rotPIDController.getSetpoint().position);
        //SmartDashboard.putNumber("GoToNote angle", m_drive.getAngle().getRadians());

        double rotSpeed = MathUtil.clamp(
            m_rotPIDController.calculate(m_drive.getAngle().getRadians())+m_rotPIDController.getSetpoint().velocity,
            -constants.kMaxAngularSpeed,
            constants.kMaxAngularSpeed);

        //SmartDashboard.putNumber("sqrt(ta)", Math.sqrt(ta));
        SmartDashboard.putNumber("transAngle", transAngle);
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);
            
        m_drive.Drive(xSpeed, ySpeed, rotSpeed, true);

        //SmartDashboard.putNumber("vision x pid setpoint", m_xPIDController.getSetpoint().position);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double wrapAngle(double angle) {
        return (angle+Math.PI)%(2*Math.PI)-Math.PI;
    }

}