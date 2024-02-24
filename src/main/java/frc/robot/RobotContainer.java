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
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.ClimberMoveTo;
import frc.robot.commands.GoToNote;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TurretMode;
import frc.robot.commands.ShooterCommands.ShooterManualAngleControl;
import frc.robot.commands.ShooterCommands.SetShooterRPM;
import frc.robot.subsystems.Breakbeam;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.OECTrigger;
import frc.robot.constants.State;
import frc.robot.constants.Direction;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import frc.robot.JoystickMath;
import frc.robot.commands.ClimberManualControl;
import frc.robot.commands.GoToAndIntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.SearchForAndGoToAndIntakeNote;
import frc.robot.commands.IntakeNoteToShooter;

public class RobotContainer {
  private final Limelight m_shooterLimelight = new Limelight("limelight");
  private final Limelight m_noteLimelight = new Limelight("limelight");
  private final Drivetrain m_robotDrive = new Drivetrain(60, m_shooterLimelight);
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandJoystick m_secondaryController = new CommandJoystick(1);

  // private Intake intake;
   private Shooter m_shooter = new Shooter(30,31,32,33,0,0,1,constants.kStartAngle,false);
  // private Climber m_climber;

  // public State m_state;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // NamedCommands.registerCommand("GoToNote", new GoToNote(m_robotDrive, m_noteLimelight, intake));
    // NamedCommands.registerCommand("IntakeNote", new IntakeNote(
    //   intake,
    //   intermediate,
    //   m_secondaryController.button(constants.kShooterOrElevatorButton)::getAsBoolean,
    //   this::updateState,
    //   this::getState));
    // NamedCommands.registerCommand("GoToAndIntakeNote", new GoToAndIntakeNote(
    //   m_robotDrive,
    //   m_noteLimelight,
    //   intake, intermediate,
    //   m_secondaryController.button(constants.kShooterOrElevatorButton)::getAsBoolean,
    //   this::updateState,
    //   this::getState));
    // NamedCommands.registerCommand("SearchForAndGoToAndIntakeNote", new SearchForAndGoToAndIntakeNote(
    //   m_robotDrive,
    //   m_noteLimelight,
    //   7.5,
    //   true,
    //   intake,
    //   intermediate,
    //   m_secondaryController.button(constants.kShooterOrElevatorButton)::getAsBoolean,
    //   this::updateState,
    //   this::getState));
    // NamedCommands.registerCommand("Shoot", new Shoot(m_shooter));

    // m_state = State.CLEAR;

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
                    JoystickMath.convert(m_driverController.getLeftY(), 2, 0.1, 1),
                    JoystickMath.convert(m_driverController.getLeftX(), 2, 0.1, 1),
                    JoystickMath.convert(m_driverController.getRightX(), 2, 0.1, 1),
                    true),
            m_robotDrive));

  }

  private void configureButtonBindings() {
    /*m_driverController.a().onTrue(new TurretMode(
      m_robotDrive, 
      m_shooter,
      m_shooterLimelight, 
      -0.0381,
      5.5479, 
      () -> -m_driverController.getLeftY(), 
      () -> -m_driverController.getLeftX(), 
      () -> -m_driverController.getRightX()));*/
  
    // m_driverController.b().onTrue(Commands.runOnce(() -> {}, m_robotDrive));
    // m_driverController.x().onTrue(
    //   new GoToAndIntakeNote(
    //     m_robotDrive,
    //     m_noteLimelight,
    //     intake,
    //     m_shooter);

    // TEST AMP LINE UP
    // m_driverController.y().onTrue(
    //   AutoBuilder.pathfindThenFollowPath(
    //     PathPlannerPath.fromPathFile("AmpPath"),
    //     new PathConstraints(
    //       2.0, 0.4,
    //       constants.kMaxAngularSpeed, constants.kMaxAngularAcceleration),
    //     0.0)); // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.

    // m_driverController.rightBumper().onTrue(
    //   new Shoot(
    //     );

    // secondary controller

    // manual forwards/reverse
    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean))
      .everyTimeItsTrue(
        new ConditionalCommand(
          Commands.runOnce(() -> m_shooter.setReverse(true)),
          Commands.runOnce(() -> m_shooter.setReverse(false)),
          m_secondaryController.button(constants.kForwardsOrReverseButton)::getAsBoolean));

    // manual shooter angle control
    Command shooterManualAngleCommand = new ShooterManualAngleControl(
                                          m_shooter,
                                          () -> MathUtil.applyDeadband(m_secondaryController.getX(), 0.1));
    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean))
      .and(() -> !shooterManualAngleCommand.isScheduled())
        .everyTimeItsTrue(shooterManualAngleCommand);

    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean)).negate()
      .and(() -> m_shooter.getCurrentCommand() == null)
        .everyTimeItsFalse(Commands.runOnce(() -> {m_shooter.m_Shooter1.set(0.0);
                                                    m_shooter.m_Shooter2.set(0.0);}));
    
    // manual shooter flywheel control
    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean))
      .and(m_secondaryController.button(constants.kShooterButton)::getAsBoolean)
        .everyTimeItsTrue(Commands.runOnce(() -> {m_shooter.setRPM(constants.kShooterDefaultRPM);}));

    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean))
      .and(() -> !m_secondaryController.button(constants.kShooterButton).getAsBoolean())
        .everyTimeItsTrue(Commands.runOnce(() -> {m_shooter.m_Shooter1.set(0.0);
                                                    m_shooter.m_Shooter2.set(0.0);}));

    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean)).negate()
      .and(() -> m_shooter.getCurrentCommand() == null)
        .everyTimeItsFalse(Commands.runOnce(() -> {m_shooter.m_Shooter1.set(0.0);
                                                    m_shooter.m_Shooter2.set(0.0);}));
    
    // intake note to shooter
    Command intakeNoteToShooter = new IntakeNoteToShooter(m_shooter);
    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean))
      .and(m_secondaryController.button(constants.kIntakeToShooterButton)::getAsBoolean)
        .whileTrue(intakeNoteToShooter);
    
    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean))
      .and(() -> !intakeNoteToShooter.isScheduled() && !m_secondaryController.button(constants.kEjectButton).getAsBoolean())
        .everyTimeItsTrue(Commands.runOnce(() -> m_shooter.disableFeeder()));

    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean)).negate()
      .and(() -> m_shooter.getCurrentCommand() == null)
        .everyTimeItsFalse(Commands.runOnce(() -> m_shooter.disableFeeder()));
    
    // manual shooter feeder control
    (new OECTrigger(m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean))
      .and(m_secondaryController.button(constants.kEjectButton)::getAsBoolean)
        .everyTimeItsTrue((Commands.runOnce(() -> {m_shooter.enableFeeder();})));

    // m_secondaryController.button(constants.kClimberButton).onTrue(
    //   new ConditionalCommand(
    //     new ClimberMoveTo(m_climber, constants.kClimberHighDefault),
    //     Commands.runOnce(() -> {}),
    //     m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean)); // climber up

    // m_secondaryController.button(constants.kClimberButton).onFalse(
    //   new ConditionalCommand(
    //     new ClimberMoveTo(m_climber, 0.0),
    //     Commands.runOnce(() -> {}),
    //     m_secondaryController.button(constants.kAutomaticOrManualButton)::getAsBoolean)); // climber down

    // m_secondaryController.button(constants.kAutomaticOrManualButton).onTrue(Commands.runOnce(() -> {}, m_climber)); // turn off manual climber control
    // m_secondaryController.button(constants.kAutomaticOrManualButton).onFalse(new ClimberManualControl(m_climber, () -> m_secondaryController.getRawAxis(constants.kClimberJoystickAxis))); // manual climber control
  }

  public void updateState(){
    // if (intake.BBisTripped() ||
    //     m_shooter.BBisTripped() ||
    //     m_state != State.CLEAR){

    //     //Loadig Bay Conditions
    //     if(intake.BBisTripped()){
    //       m_state = State.INTAKE;
    //     }
    //     else if(m_shooter.BBisTripped()){
    //       m_state = State.SHOOTER;
    //     }
    //     //Clear or SYS condition
    //     else if (m_state == State.INTAKE && !intake.BBisTripped()) {
    //       if (intake.getDirection() == Direction.FORWARD) {
    //         m_state = State.SYSTEM;
    //       } else if(intake.getDirection() == Direction.REVERSE){
    //         m_state = State.CLEAR;
    //       }
    //       else m_state = State.CLEAR;
    //     }
    //     else if (m_state == State.SHOOTER && !m_shooter.BBisTripped()) {
    //       if (m_shooter.getDirection() == Direction.FORWARD) {
    //         m_state = State.CLEAR;
    //       }
    //       else if(m_shooter.getDirection() == Direction.REVERSE){
    //         m_state = State.SYSTEM;
    //       }
    //       else m_state = State.CLEAR;
    //     }
    // }
  }

  // public State getState() {
  //   return m_state;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
      return new PathPlannerAuto("note detection auto");
    }
}























