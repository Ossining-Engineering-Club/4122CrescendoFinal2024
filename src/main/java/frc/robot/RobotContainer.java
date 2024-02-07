// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.GoToNote;
import frc.robot.commands.TurretMode;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Limelight m_shooterLimelight = new Limelight("limelight");
  private final Limelight m_elevatorLimelight = new Limelight("tochange");
  private final Limelight m_noteLimelight = new Limelight("limelight");
  private final Drivetrain m_robotDrive = new Drivetrain(13, m_shooterLimelight, m_elevatorLimelight);
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  CommandXboxController m_driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("GoToNote", new GoToNote(m_robotDrive, m_noteLimelight));

    m_shooterLimelight.setPipeline(0);
    m_noteLimelight.setPipeline(0);
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> 
                m_robotDrive.Drive(
                    0.4*Math.pow(MathUtil.applyDeadband(-m_driverController.getLeftY(), constants.kControllerDeadband), 2)*Math.signum(-m_driverController.getLeftY())*constants.kMaxSpeed,
                    0.4*Math.pow(MathUtil.applyDeadband(-m_driverController.getLeftX(),constants.kControllerDeadband), 2)*Math.signum(-m_driverController.getLeftX())*constants.kMaxSpeed,
                    0.4*Math.pow(MathUtil.applyDeadband(-m_driverController.getRightX(),constants.kControllerDeadband), 2)*Math.signum(-m_driverController.getRightX())*constants.kMaxAngularSpeed,
                    true),
            m_robotDrive));    
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_driverController.a().onTrue(new TurretMode(
      m_robotDrive, 
      m_shooterLimelight, 
      -8.308975,
      1.442593, 
      () -> -m_driverController.getLeftY(), 
      () -> -m_driverController.getLeftX(), 
      () -> -m_driverController.getRightX()));
    m_driverController.b().onTrue(new GoToNote(m_robotDrive, m_noteLimelight));
    m_driverController.x().onTrue(Commands.runOnce(() -> {}, m_robotDrive));
        
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("note detection auto");
    }
}























