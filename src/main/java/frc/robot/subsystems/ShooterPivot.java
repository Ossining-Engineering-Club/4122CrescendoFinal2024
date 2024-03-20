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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants;
import frc.robot.constants.Direction;

public class ShooterPivot extends SubsystemBase {

    public CANSparkMax m_Angle;
    //private RelativeEncoder e_Angle;
    public PIDController AnglePIDController = new PIDController(constants.kAnglePIDGains[0],
                                                                constants.kAnglePIDGains[1],
                                                                constants.kAnglePIDGains[2]);

    public double v_startAngle = 58.0;
    private DutyCycleEncoder e_Angle;
    private boolean is_backward;


  public ShooterPivot(
    int motorAnglePort,
    double startangle,
    boolean isAngleInverted){
        m_Angle = new CANSparkMax(motorAnglePort,MotorType.kBrushless);
        is_backward=false;
        v_startAngle = startangle;
        m_Angle.setInverted(isAngleInverted);
        e_Angle =  new DutyCycleEncoder(0);//m_Angle.getEncoder();
        e_Angle.setDistancePerRotation(360.0 / 3.0);
        // e_Angle.setPositionConversionFactor(constants.kAngleRatio);
        // e_Angle.setVelocityConversionFactor(constants.kAngleRatio / 60.0);

        // this.resetEncoders();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter angle",this.getAngle());
        // SmartDashboard.putNumber("shooter angle setpoint", AnglePIDController.getSetpoint());
        Logger.recordOutput("ShooterAngle", this.getAngle());
    
    }

    // public void resetEncoders(){
    //     e_Angle.setPositionOffset(constants.absAngleOffset);
    // }

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
        return e_Angle.getDistance() - constants.absAngleOffset;
    }

    public void stopAngle() {
        m_Angle.set(0.0);
    }
}
