package frc.robot.subsystems;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
public class OECNavX {
    AHRS gyro;
    public OECNavX(){
        gyro = new AHRS(SPI.Port.kMXP);
        
        
    }

    public void ResetYaw(){
        
        gyro.zeroYaw();
        gyro.reset();
    }
    public double GetYaw(){
        return -gyro.getYaw();
    }

}
