package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.DoubleSupplier;
import frc.robot.constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import java.lang.Math;

public class TurretMode extends Command {
    private final Drivetrain m_drive;
    private final Limelight m_limelight;
    private final DoubleSupplier m_StickX;
    private final DoubleSupplier m_StickY;
    private final DoubleSupplier m_StickYaw;
    private final double m_GoalX;
    private final double m_GoalY;

    // Slew rate limiters to cap the max acceleration of the inputs (joysticks) which makes them smoother
    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(constants.kMaxAcceleration);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(constants.kMaxAcceleration);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(constants.kMaxAngularAcceleration);

    private final ProfiledPIDController m_rotPIDController = new ProfiledPIDController(
        constants.kVisTurretPID[0],
        constants.kVisTurretPID[1],
        constants.kVisTurretPID[2],
        new TrapezoidProfile.Constraints(constants.kMaxAngularSpeed, constants.kMaxAngularAcceleration)
    );

    public TurretMode(Drivetrain drive, Limelight limelight, double goalx, double goaly, DoubleSupplier stickx, DoubleSupplier sticky, DoubleSupplier stickyaw) {
        m_drive = drive;
        m_limelight = limelight;
        m_StickX = stickx;
        m_StickY = sticky;
        m_StickYaw = stickyaw;
        m_GoalX = goalx;
        m_GoalY = goaly;

        addRequirements(m_drive);

        m_rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        m_rotPIDController.reset(m_limelight.getBotYaw()/180*Math.PI);
    }

    @Override
    public void execute() {
        final double xSpeed = m_xSpeedLimiter.calculate(constants.kMaxSpeed*MathUtil.applyDeadband(m_StickX.getAsDouble(), constants.kControllerDeadband));
        final double ySpeed = m_ySpeedLimiter.calculate(constants.kMaxSpeed*MathUtil.applyDeadband(m_StickY.getAsDouble(), constants.kControllerDeadband));
        final double rotation = m_rotLimiter.calculate(constants.kMaxAngularSpeed*MathUtil.applyDeadband(m_StickYaw.getAsDouble(), constants.kControllerDeadband));

        if (m_limelight.hasTarget()) {
            //Pose2d robotPose = m_drive.SwerveOdometryGetPose();
            double rotPos = m_limelight.getBotYaw()/180*Math.PI;
            double rotGoal = Math.atan2((m_GoalY - m_limelight.getBotY()), (m_GoalX - m_limelight.getBotX()));

            SmartDashboard.putNumber("rotPos", rotPos);
            SmartDashboard.putNumber("rotGoal", rotGoal);

            if (Math.abs(rotPos-rotGoal) < constants.kVisTurretToleranceRadians) rotPos = rotGoal;

            m_rotPIDController.setGoal(rotGoal);
            double rotSpeed = MathUtil.clamp(m_rotPIDController.calculate(rotPos)+m_rotPIDController.getSetpoint().velocity, -constants.kMaxAngularSpeed, constants.kMaxAngularSpeed);
            SmartDashboard.putNumber("rotSpeed", rotSpeed);
            
            m_drive.Drive(0.4*xSpeed, 0.4*ySpeed, rotSpeed, true);
        }else{
           m_drive.Drive(0.4*xSpeed, 0.4*ySpeed, 0.4*rotation, true);
        }

        //SmartDashboard.putNumber("Vision Yaw PID Setpoint: ", );
    }

}
