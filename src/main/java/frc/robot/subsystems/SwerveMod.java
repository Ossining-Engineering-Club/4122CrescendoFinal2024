package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants;

public class SwerveMod {

    private CANSparkMax m_Rotator;
    private RelativeEncoder e_Rotator;
    public TalonFX m_Drive;
    private double turningEncoderOffset;
    private double absEncoderOffset;
    private CANcoder absEncoder;
  
    TalonFXConfiguration configs = new TalonFXConfiguration();
    private double reversD = 1.0;
    public PIDController turningPIDController = new PIDController(constants.k_KRP, constants.k_KRI, constants.k_KRD);
    public SwerveModuleState optimizedState;


    public SwerveMod(int RotatorMotorNo, int DriveMotorNo, int CANCoderId, 
                        boolean ReverseDirection, double AbsEncoderOffsetConstant, 
                        boolean DriveReverse,boolean TurnReverse){
                            
        absEncoderOffset = AbsEncoderOffsetConstant;
        m_Rotator = new CANSparkMax(RotatorMotorNo,MotorType.kBrushless);
        m_Drive = new TalonFX(DriveMotorNo, "rio");
        absEncoder = new CANcoder(CANCoderId);
        configs.Feedback.SensorToMechanismRatio=1.0/constants.k_DriveEncoderPosFactor;
        m_Drive.setInverted(DriveReverse);
        m_Rotator.setInverted(TurnReverse);
        e_Rotator = m_Rotator.getEncoder();
        e_Rotator.setPositionConversionFactor(constants.k_TurnEncoderPosFactor);
        e_Rotator.setVelocityConversionFactor(constants.k_TurnEncoderVelocityFactor);
        m_Drive.getConfigurator().apply(configs);
        m_Drive.setNeutralMode(NeutralModeValue.Brake);
        turningPIDController.enableContinuousInput(
            -1.0*constants.k_PI, 1.0*constants.k_PI);

        this.ResetEncoder();
    }
    public SwerveModuleState GetState() {
        //Turning Encoder offset by addition of initial absolute encoder reading
        return new SwerveModuleState(m_Drive.getVelocity().getValue(),Rotation2d.fromRadians(e_Rotator.getPosition() + turningEncoderOffset));
      }
      
      // Get Position Method for Odometry
    public SwerveModulePosition GetPosition() {
        //Turning Encoder offset by addition of initial absolute encoder reading
        return new SwerveModulePosition(m_Drive.getPosition().getValue(),Rotation2d.fromRadians(e_Rotator.getPosition() + turningEncoderOffset));
    }
    public double GetAbsEncoderAngle(){
        double angle = absEncoder.getAbsolutePosition().getValue();
        angle *= (2*constants.k_PI);//(constants.k_PI / 180.0);
        angle-=absEncoderOffset;
        // // Pi Check:
        if(angle < -constants.k_PI) angle += (2.0 * constants.k_PI);
        else if(angle > constants.k_PI) angle -= (2.0 * constants.k_PI);
    
        return angle;
    }
    public double GetCurrentAngle(){
        
        return wrapAngle(e_Rotator.getPosition() + turningEncoderOffset);
    }
    public double wrapAngle(double angle) {
        return (angle+Math.PI)%(2*Math.PI)-Math.PI;
    }
    public void ResetEncoder(){
        turningEncoderOffset = this.GetAbsEncoderAngle();
        m_Drive.setPosition(0.0);
        e_Rotator.setPosition(0.0);
    
    }
    public void SetDesiredState(SwerveModuleState state) {
        if(Math.abs(state.speedMetersPerSecond) > 0.01){
            SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(this.GetCurrentAngle()));
            
            double turningVal = turningPIDController.calculate(this.GetCurrentAngle(), 
                                                               optimizedState.angle.getRadians());
            
            if(turningVal > 1.0) turningVal = 1.0;
            else if(turningVal < -1.0) turningVal = -1.0;
    
    
            m_Drive.setVoltage(optimizedState.speedMetersPerSecond*(1.0/constants.kMaxSpeed)*12.0); //Change to variable later
            m_Rotator.set(turningVal);
        }
        else{
            m_Drive.set(0.0);
            m_Rotator.set(0.0);
        }
    }
}
