package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// subsys imports
import frc.robot.subsystems.Intermediate;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

import java.lang.Runnable;
import java.util.function.Supplier;
import frc.robot.constants;

public class EjectNote extends Command {

    private final Intermediate m_Intermediate;
    private final Elevator m_Elevator;
    private final Intake m_Intake;
    private final Runnable updateState;
    private final Supplier getState;

    private boolean statebool;

    public EjectNote(Intermediate intermediate, Elevator elevator, Intake intake, Runnable updateState, Supplier getState) {
        m_Intermediate = intermediate;
        m_Elevator = elevator;
        m_Intake = intake;

        this.updateState = updateState;
        this.getState = getState;

        addRequirements(m_Intermediate);
        addRequirements(m_Elevator);
        addRequirements(m_Intake);
    }

    @Override
    public void initialize() {
        }

    @Override
    public void execute() {

        }

    @Override 
    public void end(boolean interrupted){
        // update condition
    }

    @Override
    public boolean isFinished(){
        statebool = false;
        return true;
    }


}