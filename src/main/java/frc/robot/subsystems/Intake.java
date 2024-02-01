package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;

import frc.robot.constants;
import frc.robot.subsystems.Breakbeam;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_intakeMotor;
    private final CANSparkMax m_pivotMotor;

    private final RelativeEncoder m_intakeEncoder;
    private final RelativeEncoder m_pivotEncoder;

    private final ProfiledPIDController m_pivotPID =
        new ProfiledPIDController(
            constants.kIntakePivotPIDGains[0],
            constants.kIntakePivotPIDGains[1],
            constants.kIntakePivotPIDGains[2],
            new TrapezoidProfile.Constraints(constants.kIntakePivotMaxVelocity, constants.kIntakePivotMaxAcceleration)
        );

    private final Breakbeam m_breakbeam;

    public Intake(int intakeMotorID, int pivotMotorID, int breakbeamReceiverPin) {
        m_intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
        m_pivotMotor = new CANSparkMax(pivotMotorID, MotorType.kBrushless);

        m_intakeEncoder = m_intakeMotor.getEncoder();
        m_pivotEncoder = m_pivotMotor.getEncoder();

        m_intakeEncoder.setPositionConversionFactor(1 / constants.kIntakeGearing); // rotations
        m_intakeEncoder.setVelocityConversionFactor(1 / constants.kIntakeGearing); // rotations per minute (rpm)

        m_pivotEncoder.setPositionConversionFactor(2*Math.PI / constants.kIntakePivotGearing); // radians
        m_pivotEncoder.setVelocityConversionFactor(2*Math.PI/60 / constants.kIntakePivotGearing); // radians per second

        m_pivotEncoder.setPosition(0);

        m_pivotPID.reset(m_pivotEncoder.getPosition());

        m_breakbeam = new Breakbeam(breakbeamReceiverPin);
    }

    @Override
    public void periodic() {
        m_pivotMotor.set(m_pivotPID.calculate(getAngle().getRadians()) + m_pivotPID.getSetpoint().velocity);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(m_pivotEncoder.getPosition());
    }

    public void setVelocity(double speedRPM) {
        m_intakeMotor.set(speedRPM/constants.kIntakeMaxSpeed);
    }

    public void setPosition(double positionRadians) {
        m_pivotPID.setGoal(positionRadians);
    }

    public void start() {
        setVelocity(constants.kIntakeDefaultSpeed);
    }

    public void stop() {
        setVelocity(0);
    }

    public void reverse() {
        setVelocity(-constants.kIntakeDefaultSpeed);
    }

    public boolean isIntaken() {
        return m_breakbeam.isTripped();
    }
}