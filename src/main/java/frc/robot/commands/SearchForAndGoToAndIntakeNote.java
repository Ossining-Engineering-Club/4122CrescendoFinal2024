package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.State;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.SearchForNote;
import frc.robot.commands.GoToAndIntakeNote;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Commands;

public class SearchForAndGoToAndIntakeNote extends SequentialCommandGroup {
    public SearchForAndGoToAndIntakeNote(
        Drivetrain drive,
        Limelight limelight,
        Intake intake,
        Shooter shooter,
        double yLimit,
        boolean isGoingPositive) {
            addCommands(
                new SearchForNote(drive, limelight, yLimit, isGoingPositive),
                new ConditionalCommand(
                    new GoToAndIntakeNote(drive, limelight, intake, shooter),
                    Commands.runOnce(() -> {}),
                    limelight::hasTarget
                )
            );
        }
}
