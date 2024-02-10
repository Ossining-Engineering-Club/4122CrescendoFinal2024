package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
// subsys imports
import frc.robot.subsystems.Intermediate;
import frc.robot.subsystems.Shooter;

public class EjectNote extends Command {

    private final Intermediate m_Intermediate;
    private final Elevator m_Elevator;
    private final Intake m_Intake;
    private final Shooter m_Shooter;

    private final Runnable updateState;
    private final Supplier<constants.State> getState;

    private boolean statebool;
    private final int RPM = 100;

    public EjectNote(Intermediate intermediate, Elevator elevator, Intake intake, Shooter shooter, Runnable updateState, Supplier<constants.State> getState) {
        m_Intermediate = intermediate;
        m_Elevator = elevator;
        m_Intake = intake;
        m_Shooter = shooter;

        this.updateState = updateState;
        this.getState = getState;

        addRequirements(m_Intermediate);
        addRequirements(m_Elevator);
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
            m_Intake.reverse();
            m_Shooter.setRPM(-RPM);
            m_Intermediate.ejectIntermediateShooter();
        }
        else if (statebool && getState.get() == constants.State.ELEVATOR) {
            m_Intake.reverse();
            m_Elevator.ElevatorMove(-RPM);
            m_Elevator.NoteElevatorMove(-RPM);
        } 
            
    }

    @Override 
    public void end(boolean interrupted){
        m_Intake.stop();
        m_Intermediate.stopIntermediate();
        m_Shooter.setRPM(0);

        m_Elevator.ElevatorMove(0);
        m_Elevator.NoteElevatorMove(0.0);

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