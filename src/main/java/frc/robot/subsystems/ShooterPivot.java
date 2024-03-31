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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants;
import frc.robot.constants.Direction;

public class ShooterPivot extends SubsystemBase {

    public CANSparkMax m_Angle;
    private DutyCycleEncoder e_Angle;
    private DigitalInput m_limitSwitch;
    public PIDController AnglePIDController = new PIDController(constants.kAnglePIDGains[0],
                                                                constants.kAnglePIDGains[1],
                                                                constants.kAnglePIDGains[2]);

    public double v_startAngle = 58.0;
    private boolean is_backward;


  public ShooterPivot(
    int motorAnglePort,
    double startangle,
    boolean isAngleInverted//,
    /*int limitSwitchPin*/){
        m_Angle = new CANSparkMax(motorAnglePort,MotorType.kBrushless);
        is_backward=false;
        v_startAngle = startangle;
        m_Angle.setInverted(isAngleInverted);
        e_Angle = new DutyCycleEncoder(0);
        
        e_Angle.setDistancePerRotation(constants.kAngleRatio);

        //m_limitSwitch = new DigitalInput(limitSwitchPin);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter angle",this.getAngle());
        SmartDashboard.putNumber("pivot set speed", m_Angle.get());
        // SmartDashboard.putNumber("shooter angle setpoint", AnglePIDController.getSetpoint());
        Logger.recordOutput("ShooterAngle", this.getAngle());
    
    }

    //returns true if setpoint is reached false otherwise
    public boolean setAngle(double Angle){
        if (Angle < constants.kShooterMinAngle) Angle = constants.kShooterMinAngle;
        else if (Angle > constants.kShooterMaxAngle) Angle = constants.kShooterMaxAngle;

        double currentangle = getAngle();

        AnglePIDController.setSetpoint(Angle);
        double adjustmentval = AnglePIDController.calculate(currentangle);
        // if (e_Angle.getPosition() >= constants.kShooterMaxAngle) {
        //     adjustmentval = Math.min(0, adjustmentval);
        // }
        // if (e_Angle.getPosition() <= constants.kShooterMinAngle || m_limitSwitch.get()) {
        //     adjustmentval = Math.max(0, adjustmentval);
        // }
        setAngleMotor(adjustmentval-0.026*Math.cos(Units.degreesToRadians(Angle))/*-0.015*/);

        if(Math.abs(Angle-currentangle) < constants.kangleTolerance){
            return true;
        }
        else{
            return false;
        }
    }

    public void setAngleMotor(double power) {
        m_Angle.set(power/*-0.015*/);
    }

    public void setReverse(boolean isOn){
        this.is_backward=isOn;
    }

    //returns true if setpoint is reached false otherwise
    public boolean isAngleReached() {
        return Math.abs(AnglePIDController.getSetpoint()-getAngle()) < constants.kangleTolerance;
    }

    public double getAngle() {
        return e_Angle.getDistance()-constants.absAngleOffset;
    }

    public void stopAngle() {
        m_Angle.set(0.0);
    }
}
