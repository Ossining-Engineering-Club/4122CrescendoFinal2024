package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intermediate;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.GoToNote;
import frc.robot.commands.IntermediateToElevator;
import frc.robot.commands.IntermediateToShooter;
import frc.robot.constants.State;
import java.util.function.BooleanSupplier;
import java.lang.Runnable;
import java.util.function.Supplier;
import frc.robot.commands.IntakeRun;

public class IntakeNote extends ParallelDeadlineGroup {
    public IntakeNote(
        Drivetrain drivetrain,
        Limelight limelight,
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
                new GoToNote(drivetrain, limelight),
                new IntakeRun(intake));
    }
}
