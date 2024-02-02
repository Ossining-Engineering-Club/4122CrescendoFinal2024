

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {


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

public class SetRPMShooter extends Command {

    public final Shooter m_shooter;

    public SetRPMShooter(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
    }

    @Override 
    public boolean isFinished(){

        return true;
    }

}
}


    