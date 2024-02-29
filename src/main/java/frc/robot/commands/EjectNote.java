package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Intake;
// subsys imports
import frc.robot.subsystems.Shooter;

public class EjectNote extends Command {

    private final Intake m_Intake;
    private final Shooter m_Shooter;

    private final Runnable updateState;
    private final Supplier<constants.State> getState;

    private boolean statebool;
    private final int RPM = 100;

    public EjectNote(Intake intake, Shooter shooter, Runnable updateState, Supplier<constants.State> getState) {
        m_Intake = intake;
        m_Shooter = shooter;

        this.updateState = updateState;
        this.getState = getState;

        addRequirements(m_Intake);
        addRequirements(m_Shooter);
    }

    @Override
    public void initialize() {
        if(getState.get() != constants.State.CLEAR) {
            statebool = true;
        }
    }

    @Override
    public void execute() {
        if(statebool && getState.get() == constants.State.SHOOTER) {
            m_Intake.setReverse(true);
            m_Intake.start();
            m_Shooter.setRPM(-RPM);
        }    
    }

    @Override 
    public void end(boolean interrupted){
        m_Intake.stop();
        m_Shooter.setRPM(0);

        updateState.run();
        // update condition
    }

    @Override
    public boolean isFinished(){
        if (getState.get() == constants.State.CLEAR) {
            statebool = false;

            return true;
        }
        return false;
        
    }


}