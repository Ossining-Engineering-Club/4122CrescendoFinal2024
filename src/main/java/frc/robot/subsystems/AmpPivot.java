package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants;

public class AmpPivot extends SubsystemBase {
    private final CANSparkMax m_pivot;
    private final RelativeEncoder e_pivot;
    private final PIDController m_pid = new PIDController(
                                            constants.kAmpPivotPIDGains[0],
                                            constants.kAmpPivotPIDGains[1],
                                            constants.kAmpPivotPIDGains[2]);

    public AmpPivot(int pivotID) {
        m_pivot = new CANSparkMax(pivotID, MotorType.kBrushless);
        e_pivot = m_pivot.getEncoder();
        e_pivot.setPositionConversionFactor(1.0/40.0 * (22.0/32.0) * 360.0);
        e_pivot.setVelocityConversionFactor(1.0/40.0 * (22.0/32.0) * 360.0 * 60.0);
        e_pivot.setPosition(0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("amp pivot angle", getAngle());
    }

    public void setAngle(double angle) {
        m_pivot.set(m_pid.calculate(e_pivot.getPosition(), angle));
    }

    public double getAngle() {
        return e_pivot.getPosition();
    }

    public boolean isAngleReached() {
        return Math.abs(e_pivot.getPosition() - m_pid.getSetpoint()) <= constants.kAmpPivotTolerance;
    }

    public void stopMotor() {
        m_pivot.set(0);
    }
}
