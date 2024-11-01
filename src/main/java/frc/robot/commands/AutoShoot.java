package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShooterCommands.AngleShooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants;

public class AutoShoot extends SequentialCommandGroup {
    public AutoShoot(Drivetrain drive, ShooterFlywheels shooterFlywheels, ShooterPivot pivot, ShooterFeeder shooterFeeder, Leds leds) {
        addCommands(
            Commands.runOnce(() -> shooterFlywheels.start(), shooterFlywheels),
            new TurretAlign(drive, pivot),
            new Shoot(shooterFlywheels, shooterFeeder, leds),
            new AngleShooter(pivot, constants.kStartAngle)
        );
    }
}
