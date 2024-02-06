package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intermediate;
import java.lang.Runnable;
import java.util.function.Supplier;
import frc.robot.constants;

public class IntermediateToElevator extends Command {

    private final Intermediate m_moveNote;
    private final Runnable updateState;
    private final Supplier getState;

    private boolean statebool;

    public IntermediateToElevator(Intermediate moveNote, Runnable updateState, Supplier getState) {
        m_moveNote = moveNote;

        this.updateState = updateState;
        this.getState = getState;

        addRequirements(m_moveNote);
    }

    @Override
    public void initialize() {
        if (getState.get() == constants.State.SYSTEM) {
            statebool = true;
        }
        }

    @Override
    public void execute() {
        if(statebool) {
             m_moveNote.moveToElevator();
        }
        }

    @Override 
    public void end(boolean interrupted){
        // update condition
        m_moveNote.stopIntermediate();
        updateState.run();
    }

    @Override
    public boolean isFinished(){
        statebool = false;
        return true;
    }


}