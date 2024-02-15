

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class ShooterCommands {

    // ONLY USE FOR TESTING
    public static class SetShooterAngle extends Command {
        private final Shooter m_shooter;
        private final double angle;

        public SetShooterAngle(Shooter shooter, double angle) {
            this.angle = angle;
            m_shooter = shooter;
            addRequirements(m_shooter);
        }

        @Override
        public void initialize() { //set position
            m_shooter.setAngle(this.angle);
        }

        @Override 
        public boolean isFinished() {
            return m_shooter.isAngleReached();
        }
    }

    public static class ShooterManualAngleControl extends Command {
        private final Shooter m_shooter;
        private final DoubleSupplier m_angleSupplier;

        public ShooterManualAngleControl(Shooter shooter, DoubleSupplier angleSupplier) {
            m_angleSupplier = angleSupplier;
            m_shooter = shooter;
            addRequirements(m_shooter);
        }

        @Override
        public void execute() {
            m_shooter.setAngle(m_shooter.getAngle()+m_angleSupplier.getAsDouble()*constants.kShooterManualAngleControlSpeedMultiplier);
        }

        @Override 
        public boolean isFinished() {
            return false;
        }
    }

    public static class SetShooterRPM extends Command {
        public final Shooter m_shooter;
        public final double m_RPM;

        public SetShooterRPM(Shooter shooter, double RPM) {
            m_shooter = shooter;
            m_RPM = RPM;

            addRequirements(m_shooter);
        }

        @Override
        public void initialize() {
            m_shooter.setRPM(m_RPM);
        }

        @Override 
        public boolean isFinished(){
            return m_shooter.isRPMReached();
        }

    }

    // TO DO
    // idk which motors are feeding
    public static class FeedToShooter extends Command {
        public FeedToShooter() {

        }
    }
}


    