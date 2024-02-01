package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intermediate;

public class IntermediateMoveNote extends Command {

    private final Intermediate m_moveNote;

    public IntermediateMoveNote(Intermediate moveNote) {
        m_moveNote = moveNote;

        addRequirements(m_moveNote);
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