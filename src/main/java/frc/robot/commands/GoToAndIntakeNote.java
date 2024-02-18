package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.IntakeNote;

public class GoToAndIntakeNote extends ParallelCommandGroup {
    public GoToAndIntakeNote(
        Drivetrain drivetrain,
        Limelight limelight,
        Intake intake,
        Intermediate intermediate,
        BooleanSupplier shooterOrElevatorSwitch,
        Runnable updateState,
        Supplier<State> getState) {
            addCommands(
                new IntakeNote(intake, intermediate, shooterOrElevatorSwitch, updateState, getState),
                new GoToNote(drivetrain, limelight)
            );
    }
}
