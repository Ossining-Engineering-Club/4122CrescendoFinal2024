package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends SequentialCommandGroup {
    public Shoot(Shooter shooter) {
        addCommands(
            new ShooterCommands.SetShooterRPM(shooter, constants.kShooterDefaultRPM),
            new ShooterCommands.FeedToShooter(),
            new ShooterCommands.SetShooterRPM(shooter, 0.0)
        );
    }
}
