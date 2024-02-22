package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.fasterxml.jackson.annotation.JsonCreator.Mode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.constants;

public class Shooter extends SubsystemBase{

    private CANSparkFlex m_Shooter1;
    private CANSparkFlex m_Shooter2;
    private CANSparkFlex m_Angle;
    private CANSparkMax m_Feeder;
    private Encoder e_Angle;
    private RelativeEncoder e_Shooter1;
    private RelativeEncoder e_Shooter2;
    public PIDController Shooter1PIDController = new PIDController(constants.kShooter1PIDGains[0],
                                                                   constants.kShooter1PIDGains[1],
                                                                   constants.kShooter1PIDGains[2]);
    public PIDController Shooter2PIDController = new PIDController(constants.kShooter2PIDGains[0],
                                                                   constants.kShooter2PIDGains[1],
                                                                   constants.kShooter2PIDGains[2]);
    public PIDController AnglePIDController = new PIDController(constants.kAnglePIDGains[0],
                                                                constants.kAnglePIDGains[1],
                                                                constants.kAnglePIDGains[2]);

    public double v_startAngle = 0;
    private boolean is_backward;


  public Shooter(
    int Flywheelport1,
    int Flywheelport2,
    int motorAnglePort,
    int feederPort,
    int angleEncoderChannelA,
    int angleEncoderChannelB,
    double startangle,
    boolean isAngleInverted){
        m_Shooter1 = new CANSparkFlex(Flywheelport1,MotorType.kBrushless);
        m_Shooter2 = new CANSparkFlex(Flywheelport2,MotorType.kBrushless);
        m_Angle = new CANSparkFlex(motorAnglePort,MotorType.kBrushless);
        m_Feeder = new CANSparkMax(feederPort,MotorType.kBrushless);
        is_backward=false;
        e_Angle = new Encoder(angleEncoderChannelA, angleEncoderChannelB, isAngleInverted);
        e_Shooter1 = m_Shooter1.getEncoder();
        e_Shooter2 = m_Shooter2.getEncoder();
        
        e_Angle.setDistancePerPulse(constants.kAngleRatio / 8192.0);
        e_Shooter1.setVelocityConversionFactor(constants.kShooterGearRatio);
        e_Shooter2.setVelocityConversionFactor(constants.kShooterGearRatio);
        

        this.resetEncoders(startangle);
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

        /*Shooter1PIDController.setSetpoint(targetRPM);
        Shooter2PIDController.setSetpoint(targetRPM);

        double adjustmentvalS1 = Shooter1PIDController.calculate(currentRPMS1);
        double adjustmentvalS2 = Shooter2PIDController.calculate(currentRPMS2);

        m_Shooter1.set(m_Shooter1.get()+adjustmentvalS1);
        m_Shooter2.set(m_Shooter2.get()+adjustmentvalS2);*/

        m_Shooter1.set(targetRPM/6000);
        m_Shooter2.set(targetRPM/6000);

        if(Math.abs(targetRPM-currentRPMS1) < constants.kRPMTolerance && 
            Math.abs(targetRPM-currentRPMS2) < constants.kRPMTolerance){
            return true;
        }else{
            return false;
        }

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
        return Math.abs(targetRPM-e_Shooter1.getVelocity()) < constants.kRPMTolerance && 
                Math.abs(targetRPM-e_Shooter2.getVelocity()) < constants.kRPMTolerance;
    }

    //returns true if setpoint is reached false otherwise
    public boolean isAngleReached() {
        return Math.abs(AnglePIDController.getSetpoint()-getAngle()) < constants.kangleTolerance;
    }

    public double getAngle() {
        return e_Angle.getDistance()+v_startAngle;
    }


    /*private CANSparkMax m_Shooter1;
    private CANSparkMax m_Shooter2;
    private CANSparkMax m_Angle;
    private RelativeEncoder e_Angle;
    private RelativeEncoder e_Shooter1;
    private RelativeEncoder e_Shooter2;
    public PIDController Shooter1PIDController = new PIDController(constants.kShooter1PIDGains[0],
                                                                   constants.kShooter1PIDGains[1],
                                                                   constants.kShooter1PIDGains[2]);
    public PIDController Shooter2PIDController = new PIDController(constants.kShooter2PIDGains[0],
                                                                   constants.kShooter2PIDGains[1],
                                                                   constants.kShooter2PIDGains[2]);
    public ProfiledPIDController AnglePIDController =
        new ProfiledPIDController(
            constants.kAnglePIDGains[0],
            constants.kAnglePIDGains[1],
            constants.kAnglePIDGains[2],
            new TrapezoidProfile.Constraints(constants.kShooterAngleMaxVelocity, constants.kShooterAngleMaxAcceleration));


  public Shooter(int Flywheelport1,int Flywheelport2, int motorAnglePort, double startangle){
        m_Shooter1 = new CANSparkMax(Flywheelport1,MotorType.kBrushless);
        m_Shooter2 = new CANSparkMax(Flywheelport2,MotorType.kBrushless);
        m_Angle = new CANSparkMax(motorAnglePort,MotorType.kBrushless);

        e_Angle = m_Angle.getEncoder();
        e_Shooter1 = m_Shooter1.getEncoder();
        e_Shooter2 = m_Shooter2.getEncoder();
        
        e_Angle.setPositionConversionFactor(constants.kAngleRatio * constants.k_PI * 2);
        e_Shooter1.setVelocityConversionFactor(constants.kShooterGearRatio);
        e_Shooter2.setVelocityConversionFactor(constants.kShooterGearRatio);

        this.resetEncoders(startangle);
    }
    public void resetEncoders(double startangle){
        e_Angle.setPosition(startangle);
        e_Shooter1.setPosition(0.0);
        e_Shooter2.setPosition(0.0);
    }
    //returns true if setpoint is reached false otherwise
    public boolean setRPM(double targetRPM){
        Shooter1PIDController.setSetpoint(targetRPM);
        Shooter2PIDController.setSetpoint(targetRPM);
        return isRPMReached();
    }

    //returns true if setpoint is reached false otherwise
    public boolean isRPMReached() {
        double targetRPM = Shooter1PIDController.getSetpoint();
        return Math.abs(targetRPM-e_Shooter1.getVelocity()) < constants.kRPMTolerance && 
                Math.abs(targetRPM-e_Shooter2.getVelocity()) < constants.kRPMTolerance;
    }
    
    //returns true if setpoint is reached false otherwise
    public boolean setAngle(double Angle) {
        AnglePIDController.setGoal(Angle);
        return isAngleReached();
    }

    public double getAngle() {
        return e_Angle.getPosition();
    }

    //returns true if setpoint is reached false otherwise
    public boolean isAngleReached() {
        return Math.abs(AnglePIDController.getGoal().position-e_Angle.getPosition()) < constants.kangleTolerance;
    }

    @Override
    public void periodic() {
        // angle pid
        m_Angle.set(AnglePIDController.calculate(e_Angle.getPosition()));

        // shooter pid
        double adjustmentvalS1 = Shooter1PIDController.calculate(e_Shooter1.getVelocity());
        double adjustmentvalS2 = Shooter2PIDController.calculate(e_Shooter2.getVelocity());

        m_Shooter1.set(m_Shooter1.get()+adjustmentvalS1);
        m_Shooter2.set(m_Shooter2.get()+adjustmentvalS2);
    }*/
}
