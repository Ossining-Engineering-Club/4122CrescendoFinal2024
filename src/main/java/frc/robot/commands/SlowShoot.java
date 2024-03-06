package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Leds;

public class SlowShoot extends SequentialCommandGroup {
    public SlowShoot(Shooter shooter, Leds leds) {
        addCommands(
            new ShooterCommands.SetShooterRPM(shooter, constants.kAmpRPM),
            new ShooterCommands.FeedToFlywheels(shooter, leds),
            new WaitCommand(.25),
            new ShooterCommands.SetShooterRPM(shooter, 0.0)
        );
    }
}
