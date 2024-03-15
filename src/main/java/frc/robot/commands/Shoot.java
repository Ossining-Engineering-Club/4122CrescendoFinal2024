package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.Leds;
import frc.robot.commands.ShooterCommands.SpinUpFlywheels;

public class Shoot extends SequentialCommandGroup {
    public Shoot(ShooterFlywheels shooterFlywheels, ShooterFeeder shooterFeeder, Leds leds) {
        addCommands(
            new ShooterCommands.SpinUpFlywheels(shooterFlywheels),
            new ShooterCommands.FeedToFlywheels(shooterFeeder, leds),
            new WaitCommand(.25),
            Commands.runOnce(() -> shooterFlywheels.stop(), shooterFlywheels)
        );
    }
}
