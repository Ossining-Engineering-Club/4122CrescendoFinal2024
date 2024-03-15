package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.Leds;

public class ShootAuto extends SequentialCommandGroup {
    public ShootAuto(ShooterFlywheels shooterFlywheels, ShooterFeeder shooterFeeder, Leds leds) {
        addCommands(
            new ShooterCommands.SpinUpFlywheels(shooterFlywheels),
            new ShooterCommands.FeedToFlywheels(shooterFeeder, leds),
            new WaitCommand(.25)
        );
    }
}
