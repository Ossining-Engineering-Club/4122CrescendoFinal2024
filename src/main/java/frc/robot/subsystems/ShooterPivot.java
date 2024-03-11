package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;
import frc.robot.constants.Direction;

public class ShooterPivot extends SubsystemBase {

    public CANSparkFlex m_Angle;
    private Encoder e_Angle;
    public PIDController AnglePIDController = new PIDController(constants.kAnglePIDGains[0],
                                                                constants.kAnglePIDGains[1],
                                                                constants.kAnglePIDGains[2]);

    public double v_startAngle = 58.0;
    private boolean is_backward;


  public ShooterPivot(
    int motorAnglePort,
    int angleEncoderChannelA,
    int angleEncoderChannelB,
    double startangle,
    boolean isAngleInverted){
        m_Angle = new CANSparkFlex(motorAnglePort,MotorType.kBrushless);
        is_backward=false;
        v_startAngle = startangle;
        e_Angle = new Encoder(angleEncoderChannelA, angleEncoderChannelB, isAngleInverted);
        
        e_Angle.setDistancePerPulse(constants.kAngleRatio/2048.0);
        this.resetEncoders(startangle);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("shooter angle",this.getAngle());
        // SmartDashboard.putNumber("shooter angle setpoint", AnglePIDController.getSetpoint());
        Logger.recordOutput("ShooterAngle", this.getAngle());
    
    }

    public void resetEncoders(double startangle){
        v_startAngle = startangle;
        e_Angle.reset();
    }

    //returns true if setpoint is reached false otherwise
    public boolean setAngle(double Angle){
        if (Angle < constants.kShooterMinAngle) Angle = constants.kShooterMinAngle;
        else if (Angle > constants.kShooterMaxAngle) Angle = constants.kShooterMaxAngle;

        double currentangle = getAngle();

        AnglePIDController.setSetpoint(Angle);
        double adjustmentval = AnglePIDController.calculate(currentangle);
        m_Angle.set(adjustmentval);

        if(Math.abs(Angle-currentangle) < constants.kangleTolerance){
            return true;
        }
        else{
            return false;
        }
    }

    public void setAngleMotor(double power) {
        m_Angle.set(power);
    }

    public void setReverse(boolean isOn){
        this.is_backward=isOn;
    }

    //returns true if setpoint is reached false otherwise
    public boolean isAngleReached() {
        return Math.abs(AnglePIDController.getSetpoint()-getAngle()) < constants.kangleTolerance;
    }

    public double getAngle() {
        return e_Angle.getDistance()+v_startAngle;
    }

    public void stopAngle() {
        m_Angle.set(0.0);
    }
}
