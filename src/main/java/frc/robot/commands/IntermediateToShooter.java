package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intermediate;
import frc.robot.constants;
import java.lang.Runnable;
import java.util.function.Supplier;
import frc.robot.constants.State;

public class IntermediateToShooter extends Command {

    private final Intermediate m_moveNote;
    private final Runnable updateState;
    private final Supplier<constants.State> getState;

    public IntermediateToShooter(Intermediate moveNote, Runnable updateState, Supplier<constants.State> getState) {
        m_moveNote = moveNote;
        this.updateState = updateState;
        this.getState = getState;

        addRequirements(m_moveNote);
    }

    @Override
    public void initialize() {
        m_moveNote.moveToShooter();
    }

    @Override 
    public void end(boolean interrupted){
        // update condition
        m_moveNote.stopIntermediate();
        updateState.run();
    }

    @Override
    public boolean isFinished(){
        return getState.get() == State.SHOOTER;
    }

}