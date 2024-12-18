package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;

public class GoToNote extends Command {
    private final Drivetrain m_drive;
    private final Limelight m_limelight;
    private final Intake m_intake;
    private final Leds m_led;
    private final Timer m_timer = new Timer();

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

    public GoToNote(Drivetrain drive, Limelight limelight, Intake intake, Leds led) {
        m_drive = drive;
        m_limelight = limelight;
        m_intake = intake;
        m_led = led;

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
        m_timer.restart();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("GoToNote gyro angle", m_drive.getAngle().getRadians());

        double transAngle;
        double overallSpeed = constants.kGoToNoteSpeed;

        if (m_limelight.hasTarget()) {
            m_led.setOrange();
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

            // restart timer if the limelight sees a target
            m_timer.restart();
        }
        else {
            transAngle = prevTransAngle;
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
        //SmartDashboard.putNumber("xSpeed", xSpeed);
        //SmartDashboard.putNumber("ySpeed", ySpeed);
        //SmartDashboard.putNumber("rotSpeed", rotSpeed);
            
        m_drive.Drive(xSpeed, ySpeed, rotSpeed, true, false);

        //SmartDashboard.putNumber("vision x pid setpoint", m_xPIDController.getSetpoint().position);
    }

    @Override
    public boolean isFinished() {
        if (m_intake.BBisTripped()) {
            SmartDashboard.putString("GoToNote finish cause", "BB was tripped");
            return true;
        }
        if (m_timer.get() > constants.kGoToNoteTimeout) {
            SmartDashboard.putString("GoToNote finish cause", "timed out");
            return true;
        }
        return false;
        //return !m_limelight.hasTarget();
        
    }

    @Override
    public void end(boolean interrupted) {
        m_led.setGreen();
         m_drive.Drive(0.0, 0.0, 0.0, true, false);
    }

    public double wrapAngle(double angle) {
        return (angle+Math.PI)%(2*Math.PI)-Math.PI;
    }

}

// package frc.robot.commands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.constants;
// import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Leds;

// public class GoToNote extends Command {
//     private final Drivetrain m_drive;
//     private final Limelight m_limelight;
//     private final Intake m_intake;
//     private final Leds m_led;
//     private final Timer m_timer = new Timer();

//     private final ProfiledPIDController m_transPIDController = new ProfiledPIDController(
//         constants.kVisionTransPIDGains[0],
//         constants.kVisionTransPIDGains[1],
//         constants.kVisionTransPIDGains[2],
//         new TrapezoidProfile.Constraints(constants.kMaxSpeed, constants.kMaxAcceleration)
//     );

//     private final ProfiledPIDController m_rotPIDController = new ProfiledPIDController(
//         constants.kVisionRotPIDGains[0],
//         constants.kVisionRotPIDGains[1],
//         constants.kVisionRotPIDGains[2],
//         new TrapezoidProfile.Constraints(constants.kMaxAngularSpeed, constants.kMaxAngularAcceleration)
//     );

//     private double prevTX;
//     private double prevTransAngle;

//     private final LinearFilter m_rotFilter = LinearFilter.movingAverage(19);

//     public GoToNote(Drivetrain drive, Limelight limelight, Intake intake, Leds led) {
//         m_drive = drive;
//         m_limelight = limelight;
//         m_intake = intake;
//         m_led = led;

//         addRequirements(m_drive);
//     }

//     @Override
//     public void initialize() {
//         m_transPIDController.reset(Math.sqrt(m_limelight.getTA()/100));
//         m_rotPIDController.reset(m_drive.getAngle().getRadians());
//         m_transPIDController.setGoal(1);
//         m_rotFilter.reset();
//         for (int i = 0; i < 50; i++) m_rotFilter.calculate(m_drive.getAngle().getRadians());
//         prevTX = -m_limelight.getTX()/180*Math.PI;
//         prevTransAngle = m_drive.getAngle().getRadians()-m_limelight.getTX()/180*Math.PI;
//         m_timer.restart();
//     }

//     @Override
//     public void execute() {
//         SmartDashboard.putNumber("GoToNote gyro angle", m_drive.getAngle().getRadians());

//         double transAngle;
//         double overallSpeed = constants.kGoToNoteSpeed;

//         if (m_limelight.hasTarget()) {
//             m_led.setOrange();
//             double tx = -m_limelight.getTX()/180*Math.PI;
//             double ty = m_limelight.getTY();
//             double ta = m_limelight.getTA()/100;

//             //if (Math.abs(tx) < constants.kVisionXToleranceRadians) tx = 0;

//             transAngle = m_drive.getAngle().getRadians()+correctOffset(tx, ty);
//             // if the limelight measurement is old, use the previous transAngle
//             if (Math.abs(tx-prevTX) < 0.00001) transAngle = prevTransAngle;
//             else {
//                 prevTX = tx;
//                 prevTransAngle = transAngle;
//             }

//             // restart timer if the limelight sees a target
//             m_timer.restart();
//         }
//         else {
//             transAngle = prevTransAngle;
//         }
        
//         double xSpeed = overallSpeed*Math.cos(transAngle);
//         double ySpeed = overallSpeed*Math.sin(transAngle);

//         m_rotPIDController.setGoal(m_rotFilter.calculate(transAngle));
//         //SmartDashboard.putNumber("GoToNote rot setpoint", m_rotPIDController.getSetpoint().position);
//         //SmartDashboard.putNumber("GoToNote angle", m_drive.getAngle().getRadians());

//         double rotSpeed = MathUtil.clamp(
//             m_rotPIDController.calculate(m_drive.getAngle().getRadians())+m_rotPIDController.getSetpoint().velocity,
//             -constants.kMaxAngularSpeed,
//             constants.kMaxAngularSpeed);

//         //SmartDashboard.putNumber("sqrt(ta)", Math.sqrt(ta));
//         SmartDashboard.putNumber("transAngle", transAngle);
//         //SmartDashboard.putNumber("xSpeed", xSpeed);
//         //SmartDashboard.putNumber("ySpeed", ySpeed);
//         //SmartDashboard.putNumber("rotSpeed", rotSpeed);

//         SmartDashboard.putNumber("gotonote xspeed", xSpeed);
//         SmartDashboard.putNumber("gotonote yspeed", ySpeed);
//         SmartDashboard.putNumber("gotonote rotspeed", rotSpeed);
            
//         //m_drive.Drive(xSpeed, ySpeed, rotSpeed, true, false);

//         //SmartDashboard.putNumber("vision x pid setpoint", m_xPIDController.getSetpoint().position);
//     }

//     @Override
//     public boolean isFinished() {
//         if (m_intake.BBisTripped()) {
//             SmartDashboard.putString("GoToNote finish cause", "BB was tripped");
//             return true;
//         }
//         if (m_timer.get() > constants.kGoToNoteTimeout) {
//             SmartDashboard.putString("GoToNote finish cause", "timed out");
//             return true;
//         }
//         return false;
//         //return !m_limelight.hasTarget();
        
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_led.setGreen();
//          m_drive.Drive(0.0, 0.0, 0.0, true, false);
//     }

//     public double wrapAngle(double angle) {
//         return (angle+Math.PI)%(2*Math.PI)-Math.PI;
//     }

//     public double convertYAngleToDistance(double angle) {
//         return 0.71+0.0374*angle+.00292*Math.pow(angle,2)+.000135*Math.pow(angle,3);
//     }

//     public double correctOffset(double tx, double ty) {
//         double camToNoteDist = convertYAngleToDistance(ty);
//         // tx += Units.degreesToRadians(constants.kNoteLimelightYawOffset);
//         // double origX = camToNoteDist * Math.cos(tx);
//         // double origY = camToNoteDist * Math.sin(tx);
//         // double correctedX = origX + constants.kNoteLimelightForwardOffset;
//         // double correctedY = origY - constants.kNoteLimelightRightOffset;
//         // double correctedTx = Math.atan2(correctedY, correctedX);
//         // SmartDashboard.putNumber("origX", origX);
//         // SmartDashboard.putNumber("origY", origY);
//         // SmartDashboard.putNumber("yaw corrected tx", tx);
//         // SmartDashboard.putNumber("correctedX", correctedX);
//         // SmartDashboard.putNumber("correctedY", correctedY);
//         // SmartDashboard.putNumber("correctedTx", correctedTx);
//         // return correctedTx;

//         double a = Math.PI/2.0-tx-Units.degreesToRadians(constants.kNoteLimelightYawOffset);
//         double centerToNoteDist = Math.sqrt(Math.pow(camToNoteDist, 2) + Math.pow(constants.kNoteLimelightRightOffset, 2))
//                                     - 2*camToNoteDist*constants.kNoteLimelightRightOffset*Math.cos(a);
//         double alpha = Math.asin(Math.max(Math.min(camToNoteDist*Math.sin(a)/centerToNoteDist, 1), -1));
//         double correctedTx = alpha-Math.PI/2.0;

//         SmartDashboard.putNumber("camToNoteDist", camToNoteDist);
//         SmartDashboard.putNumber("gotonote a", a);
//         SmartDashboard.putNumber("centerToNoteDist", centerToNoteDist);
//         SmartDashboard.putNumber("alpha", alpha);
//         SmartDashboard.putNumber("correctedTx", correctedTx);

//         return correctedTx;
//     }

// }