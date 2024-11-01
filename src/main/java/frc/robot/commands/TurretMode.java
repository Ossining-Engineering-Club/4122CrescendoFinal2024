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
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;

import java.lang.Math;

public class TurretMode extends Command {
    private final Drivetrain m_drive;
    private final ShooterFlywheels m_flywheels;
    private final ShooterPivot m_pivot;
    private final DoubleSupplier m_StickX;
    private final DoubleSupplier m_StickY;
    private final DoubleSupplier m_StickYaw;
    private double m_GoalX;
    private double m_GoalY;

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

    public TurretMode(Drivetrain drive, ShooterFlywheels flywheels, ShooterPivot pivot, DoubleSupplier stickx, DoubleSupplier sticky, DoubleSupplier stickyaw) {
        m_drive = drive;
        m_flywheels = flywheels;
        m_pivot = pivot;
        m_StickX = stickx;
        m_StickY = sticky;
        m_StickYaw = stickyaw;

        addRequirements(m_drive, m_flywheels, m_pivot);

        m_rotPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        m_rotPIDController.reset(m_drive.SwerveOdometryGetPose().getRotation().getRadians());

        if (constants.k_isRed){
            m_GoalX = constants.kRedSpeakerX;
            m_GoalY = constants.kRedSpeakerY;
        }else{
            m_GoalX = constants.kBlueSpeakerX;
            m_GoalY = constants.kBlueSpeakerY;            
        }
    }

    @Override
    public void execute() {
        final double xSpeed = m_StickX.getAsDouble();
        final double ySpeed = m_StickY.getAsDouble();
        final double rotation = m_StickYaw.getAsDouble();

        // turn on the shooter
        //m_flywheels.setRPM(constants.kShooterDefaultRPM);
        
        Pose2d robotPose = m_drive.SwerveOdometryGetPose();
        double rotPos = robotPose.getRotation().getRadians();
        double rotGoal = wrapAngle(Math.PI+Math.atan2((m_GoalY - robotPose.getY()), (m_GoalX - robotPose.getX())));

        SmartDashboard.putNumber("rotPos", rotPos);
        SmartDashboard.putNumber("rotGoal", rotGoal);

        if (Math.abs(rotPos-rotGoal) < constants.kVisTurretToleranceRadians) rotPos = rotGoal;

        m_rotPIDController.setGoal(rotGoal);
        double rotSpeed = MathUtil.clamp(m_rotPIDController.calculate(rotPos)+m_rotPIDController.getSetpoint().velocity, -constants.kMaxAngularSpeed, constants.kMaxAngularSpeed);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);
        
        m_drive.Drive(xSpeed, ySpeed, rotSpeed, true, true);

        // setting shooter angle
        double distFromTarget = Math.sqrt(Math.pow(m_GoalX - robotPose.getX(), 2) + Math.pow(m_GoalY - robotPose.getY(), 2));
        SmartDashboard.putNumber("distance from speaker", distFromTarget);
        m_pivot.setAngle(convertDistanceToShooterAngle(distFromTarget), false);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.Drive(0.0, 0.0, 0.0, true, true);
        m_pivot.stopAngle();
    }

    // TO DO
    public double convertDistanceToShooterAngle(double dist) {
        //return 3.1574 * Math.pow(dist,2) - 28.635*dist + 84.562;
        return 4.5255*Math.pow(dist, 2) - 33.4663*dist + 82.7279;
    }

    public double wrapAngle(double angle) {
        return (angle+Math.PI)%(2*Math.PI)-Math.PI;
    }
}