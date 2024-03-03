package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
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

public class Shooter extends SubsystemBase {

    public CANSparkFlex m_Shooter1;
    public CANSparkFlex m_Shooter2;
    public CANSparkFlex m_Angle;
    private CANSparkMax m_Feeder;
    private Encoder e_Angle;
    private RelativeEncoder e_Shooter1;
    private RelativeEncoder e_Shooter2;
    private Breakbeam m_breakbeam;
    //No KA because very little
    // public SimpleMotorFeedforward Shooter1PIDController = new SimpleMotorFeedforward(kS, kV);
    // public SimpleMotorFeedforward Shooter2PIDController = new SimpleMotorFeedforward(kS, kV);
    public PIDController Shooter1PIDController = new PIDController(constants.kShooter1PIDGains[0],
                                                                   constants.kShooter1PIDGains[1],
                                                                   constants.kShooter1PIDGains[2]);
    public PIDController Shooter2PIDController = new PIDController(constants.kShooter2PIDGains[0],
                                                                   constants.kShooter2PIDGains[1],
                                                                   constants.kShooter2PIDGains[2]);
    public PIDController AnglePIDController = new PIDController(constants.kAnglePIDGains[0],
                                                                constants.kAnglePIDGains[1],
                                                                constants.kAnglePIDGains[2]);

    public double v_startAngle = 58.0;
    private boolean is_backward;


  public Shooter(
    int Flywheelport1,
    int Flywheelport2,
    int motorAnglePort,
    int feederPort,
    int breakbeamPin,
    int angleEncoderChannelA,
    int angleEncoderChannelB,
    double startangle,
    boolean isAngleInverted){
        m_Shooter1 = new CANSparkFlex(Flywheelport1,MotorType.kBrushless);
        m_Shooter2 = new CANSparkFlex(Flywheelport2,MotorType.kBrushless);
        m_Angle = new CANSparkFlex(motorAnglePort,MotorType.kBrushless);
        m_Feeder = new CANSparkMax(feederPort,MotorType.kBrushless);
        m_breakbeam = new Breakbeam(breakbeamPin);
        is_backward=false;
        e_Angle = new Encoder(angleEncoderChannelA, angleEncoderChannelB, isAngleInverted);
        e_Shooter1 = m_Shooter1.getEncoder();
        e_Shooter2 = m_Shooter2.getEncoder();
        
        e_Angle.setDistancePerPulse(constants.kAngleRatio/2048.0);
        e_Shooter1.setVelocityConversionFactor(constants.kShooterGearRatio);
        e_Shooter2.setVelocityConversionFactor(constants.kShooterGearRatio);
        

        this.resetEncoders(startangle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter beambreak voltage", m_breakbeam.getVoltage());
        SmartDashboard.putBoolean("shooter beambreak isTripped", m_breakbeam.isTripped());
        SmartDashboard.putNumber("shooter angle",this.getAngle());
        SmartDashboard.putNumber("shooter angle setpoint", AnglePIDController.getSetpoint());
        SmartDashboard.putNumber("shooter RPM", e_Shooter2.getVelocity());
    }

    public void resetEncoders(double startangle){
        v_startAngle = startangle;
        e_Angle.reset();
        e_Shooter1.setPosition(0.0);
        e_Shooter2.setPosition(0.0);
    }
    //returns true if setpoint is reached false otherwise
    public boolean setRPM(double targetRPM){
        double currentRPMS1 = e_Shooter1.getVelocity();
        double currentRPMS2 = e_Shooter2.getVelocity();

        Shooter1PIDController.setSetpoint(targetRPM);
        Shooter2PIDController.setSetpoint(targetRPM);

        double adjustmentvalS1 = Shooter1PIDController.calculate(currentRPMS1);
        double adjustmentvalS2 = Shooter2PIDController.calculate(currentRPMS2);

        m_Shooter1.set(m_Shooter1.get()+adjustmentvalS1);
        m_Shooter2.set(m_Shooter2.get()+adjustmentvalS2);

        // SmartDashboard.putNumber("Shooter Setpoint", targetRPM);
        // SmartDashboard.putNumber("Shooter RPM", e_Shooter1.getVelocity());
        // SmartDashboard.putNumber("Shooter Angle", getAngle());

        // m_Shooter1.set(targetRPM/6000);
        // m_Shooter2.set(targetRPM/6000);

        if(Math.abs(Shooter1PIDController.getSetpoint()-currentRPMS1) < constants.kRPMTolerance && 
            Math.abs(Shooter2PIDController.getSetpoint()-currentRPMS2) < constants.kRPMTolerance){
            return true;
        }else{
            return false;
        }
    }

    public void stopFlywheels() {
        m_Shooter1.set(0);
        m_Shooter2.set(0);
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

    public void enableFeeder() {
        if(this.is_backward)
            m_Feeder.set(-constants.kShooterFeederSpeed);
        else
            m_Feeder.set(constants.kShooterFeederSpeed);
    }
    public void disableFeeder() {
        m_Feeder.set(0.0);
    }
    public void setReverse(boolean isOn){
        this.is_backward=isOn;
    }
    public boolean isRPMReached() {
        double targetRPM = Shooter1PIDController.getSetpoint();
        return Math.abs(Shooter1PIDController.getSetpoint()-e_Shooter1.getVelocity()) < constants.kRPMTolerance && 
                Math.abs(Shooter2PIDController.getSetpoint()-e_Shooter2.getVelocity()) < constants.kRPMTolerance;
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

    public boolean BBisTripped() {
        return m_breakbeam.isTripped();
    }

    public Direction getDirection() {
        if (m_Feeder.get() == 0.0) return Direction.STOPPED;
        if (is_backward) return Direction.REVERSE;
        return Direction.FORWARD;
    }
}
