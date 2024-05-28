package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.Leds;

public class AmpShoot extends SequentialCommandGroup {
    public AmpShoot(ShooterFeeder shooterFeeder, Leds leds) {
        addCommands(
            Commands.runOnce(() -> shooterFeeder.setReverse(true)),
            Commands.runOnce(() -> shooterFeeder.enableAmpFeeder())
            // new WaitCommand(1.0),
            // Commands.runOnce(() -> shooterFeeder.disableFeeder(), shooterFeeder),
            // Commands.runOnce(() -> shooterFeeder.setReverse(false))
        );
    }
}
