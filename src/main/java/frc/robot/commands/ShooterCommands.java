

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {

    // ONLY USE FOR TESTING
    public class AngleShooter extends Command {

        private final Shooter m_shooter;
        private final double angle;
        private boolean isdone;

        public AngleShooter(Shooter shooter, double angle) {
            this.angle = angle;
            this.isdone = false;
            m_shooter = shooter;
            addRequirements(m_shooter);

        }
        @Override
        public void execute() { //set position
            this.isdone=m_shooter.setAngle(this.angle);
        }

        @Override 
        public boolean isFinished() { // set  position
            return this.isdone;
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
        public void execute() {
            if (m_RPM == 0.0) m_shooter.stopFlywheels();
            else m_shooter.setRPM(m_RPM);
        }

        @Override 
        public boolean isFinished(){
            if (m_RPM == 0.0) return true;
            return m_shooter.isRPMReached();
        }

    }

    public static class FeedToFlywheels extends Command {
        
        public final Shooter m_Shooter;

        public FeedToFlywheels(Shooter shooter) {
            m_Shooter = shooter;

            addRequirements(m_Shooter);
        }

        @Override
        public void execute() {
            m_Shooter.enableFeeder();
        }

        @Override
        public boolean isFinished() {
            return !m_Shooter.BBisTripped();
        }
    }
}


    