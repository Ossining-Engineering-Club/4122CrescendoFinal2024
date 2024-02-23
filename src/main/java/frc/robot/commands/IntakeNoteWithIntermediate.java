package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intermediate;
import frc.robot.constants.State;

public class IntakeNoteWithIntermediate extends ParallelDeadlineGroup {
    public IntakeNoteWithIntermediate(
        Intake intake,
        Intermediate intermediate,
        BooleanSupplier shooterOrElevatorSwitch,
        Runnable updateState,
        Supplier<State> getState) {
            super(
                new ConditionalCommand(
                    new IntermediateToShooter(intermediate, updateState, getState),
                    new IntermediateToElevator(intermediate, updateState, getState),
                    shooterOrElevatorSwitch),
                new IntakeRun(intake)
            );
    }
}
