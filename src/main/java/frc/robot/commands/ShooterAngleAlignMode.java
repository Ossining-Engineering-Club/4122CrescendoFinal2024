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
import frc.robot.subsystems.ShooterPivot;
import java.lang.Math;

public class ShooterAngleAlignMode extends Command {
    private final Drivetrain m_drive;
    private final ShooterPivot m_shooter;
    private final Limelight m_limelight;
    private final double m_GoalX;
    private final double m_GoalY;
    private boolean m_isDone = false;

    public ShooterAngleAlignMode(Drivetrain drive, ShooterPivot shooter, Limelight limelight, double goalx, double goaly) {
        m_drive = drive;
        m_shooter = shooter;
        m_limelight = limelight;
        m_GoalX = goalx;
        m_GoalY = goaly;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        //m_rotPIDController.reset(m_limelight.getBotYaw()/180*Math.PI);

        // turn on the shooter
        //m_shooter.setRPM(constants.kShooterDefaultRPM);
    }

    @Override
    public void execute() {
      //  if (m_limelight.hasTarget()) {
            Pose2d robotPose = m_drive.SwerveOdometryGetPose();

            // setting shooter angle
            //double distFromTarget = Math.sqrt(Math.pow(m_GoalX - m_limelight.getBotX(), 2) + Math.pow(m_GoalY - m_limelight.getBotY(), 2));
            double distFromTarget = Math.sqrt(Math.pow(m_GoalX - robotPose.getX(), 2) + Math.pow(m_GoalY - robotPose.getY(), 2));
            SmartDashboard.putNumber("distance from speaker", distFromTarget);
            m_shooter.setAngle(convertDistanceToShooterAngle(distFromTarget), false);

        // }
        // else {
        //     m_drive.Drive(0.0, 0.0, 0.0, true);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.Drive(0.0, 0.0, 0.0, true, false);
        m_shooter.stopAngle();
    }

    // TO DO
    public double convertDistanceToShooterAngle(double dist) {
        return 3.1574 * Math.pow(dist,2) - 28.635*dist + 84.562;
    }

    public double wrapAngle(double angle) {
        return (angle+Math.PI)%(2*Math.PI)-Math.PI;
    }
}
