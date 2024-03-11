package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.Leds;

public class Shoot extends SequentialCommandGroup {
    public Shoot(ShooterFlywheels shooterFlywheels, ShooterFeeder shooterFeeder, Leds leds) {
        addCommands(
            new ShooterCommands.SetShooterRPM(shooterFlywheels, constants.kShooterDefaultRPM),
            new ShooterCommands.FeedToFlywheels(shooterFeeder, leds),
            new WaitCommand(.25),
            new ShooterCommands.SetShooterRPM(shooterFlywheels, 0.0)
        );
    }
}
