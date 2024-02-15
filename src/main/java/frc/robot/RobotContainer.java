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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.commands.ClimberMoveTo;
import frc.robot.commands.ElevatorExtend;
import frc.robot.commands.GoToNote;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TurretMode;
import frc.robot.commands.ShooterCommands.ShooterManualAngleControl;
import frc.robot.commands.ShooterCommands.SetShooterRPM;
import frc.robot.subsystems.Breakbeam;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.constants.State;
import frc.robot.constants.Direction;
import frc.robot.subsystems.Intermediate;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.JoystickMath;
import frc.robot.commands.ClimberManualControl;
import frc.robot.commands.ElevatorManualControl;

public class RobotContainer {
  private final Limelight m_shooterLimelight = new Limelight("limelight");
  private final Limelight m_elevatorLimelight = new Limelight("tochange");
  private final Limelight m_noteLimelight = new Limelight("limelight");
  private final Drivetrain m_robotDrive = new Drivetrain(13, m_shooterLimelight, m_elevatorLimelight);
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandXboxController m_secondaryController = new CommandXboxController(1);

  private Intermediate intermediate;
  private Intake intake;
  private Shooter m_shooter;
  private Elevator m_elevator;
  private Climber m_climber;

  public State m_state;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("GoToNote", new GoToNote(m_robotDrive, m_noteLimelight));

    m_state = State.CLEAR;

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
                    0.2*JoystickMath.convert(m_driverController.getLeftY(), 2, 0.1, 1),
                    0.2*JoystickMath.convert(m_driverController.getLeftX(), 2, 0.1, 1),
                    0.2*JoystickMath.convert(m_driverController.getRightX(), 2, 0.1, 1),
                    true),
            m_robotDrive));
  }

  private void configureButtonBindings() {
    m_driverController.a().onTrue(new TurretMode(
      m_robotDrive, 
      m_shooter,
      m_shooterLimelight, 
      -0.0381,
      5.5479, 
      () -> -m_driverController.getLeftY(), 
      () -> -m_driverController.getLeftX(), 
      () -> -m_driverController.getRightX()));
    m_driverController.b().onTrue(new GoToNote(m_robotDrive, m_noteLimelight));
    /*m_driverController.y().onTrue(AutoBuilder.pathfindToPose(
      new Pose2d(4.441, 4.441, Rotation2d.fromDegrees(180)),
      new PathConstraints(
        2.0, 4.0,
        constants.kMaxAngularSpeed, constants.kMaxAngularAcceleration),
      0.0,
      0.0
    ));*/
    m_driverController.x().onTrue(Commands.runOnce(() -> {}, m_robotDrive));

    // secondary controller
    m_secondaryController.a().onTrue(Commands.runOnce(() -> {})); // forwards/reverse
    m_secondaryController.a().onFalse(Commands.runOnce(() -> {})); // forwards/reverse
    m_secondaryController.b().onTrue(new SetShooterRPM(m_shooter, constants.kShooterDefaultRPM)); // shooter on
    m_secondaryController.b().onFalse(new SetShooterRPM(m_shooter, 0.0)); // shooter off
    m_secondaryController.x().onTrue(Commands.runOnce(() -> {})); // shooter/elevator
    m_secondaryController.x().onFalse(Commands.runOnce(() -> {})); // shooter/elevator

    m_secondaryController.y().onTrue(
      new ConditionalCommand(
        new ElevatorExtend(m_elevator, constants.kElevatorHighDefault),
        Commands.runOnce(() -> {}),
        m_secondaryController.rightBumper()::getAsBoolean)); // elevator up

    m_secondaryController.y().onFalse(
      new ConditionalCommand(
        new ElevatorExtend(m_elevator, 0.0),
        Commands.runOnce(() -> {}),
        m_secondaryController.rightBumper()::getAsBoolean)); // elevator down

    m_secondaryController.leftBumper().onTrue(
      new ConditionalCommand(
        new ClimberMoveTo(m_climber, constants.kClimberHighDefault),
        Commands.runOnce(() -> {}),
        m_secondaryController.rightBumper()::getAsBoolean)); // climber up

    m_secondaryController.leftBumper().onFalse(
      new ConditionalCommand(
        new ClimberMoveTo(m_climber, 0.0),
        Commands.runOnce(() -> {}),
        m_secondaryController.rightBumper()::getAsBoolean)); // climber down

    m_secondaryController.rightBumper().onTrue(Commands.runOnce(() -> {})); // automatic
    m_secondaryController.rightBumper().onTrue(Commands.runOnce(() -> {}, m_climber)); // turn off manual climber control
    m_secondaryController.rightBumper().onFalse(new ClimberManualControl(m_climber, m_secondaryController::getRightY)); // manual climber control
   
    m_secondaryController.rightBumper().onFalse(
      new ConditionalCommand(
        new ShooterManualAngleControl(m_shooter, m_secondaryController::getLeftY),
        new ElevatorManualControl(m_elevator, m_secondaryController::getLeftY),
        m_secondaryController.x()::getAsBoolean)); // manual shooter/elevator control

    m_secondaryController.rightBumper().onFalse(Commands.runOnce(() -> {})); // manual
    m_secondaryController.leftStick().onTrue(Commands.runOnce(() -> {})); // eject
  }

  public void updateState(){
    if (intake.BBisTripped() ||
        intermediate.ShooterBBisTripped() ||
        intermediate.ElevatorBBisTripped() || m_state != State.CLEAR){

        //Loadig Bay Conditions
        if(intake.BBisTripped()){
          m_state = State.INTAKE;
        }
        else if(intermediate.ElevatorBBisTripped()){
          m_state = State.ELEVATOR;
        }
        else if(intermediate.ShooterBBisTripped()){
          m_state = State.SHOOTER;
        }
        //Clear or SYS condition
        else if (m_state == State.INTAKE && !intake.BBisTripped()) {
          if (intake.getDirection() == Direction.FORWARD) {
            m_state = State.SYSTEM;
          } else if(intake.getDirection() == Direction.REVERSE){
            m_state = State.CLEAR;
          }
          else m_state = State.CLEAR;
        }
        else if (m_state == State.ELEVATOR && !intermediate.ElevatorBBisTripped()) {
          if (intermediate.getElevatorDirection() == Direction.FORWARD) {
            m_state = State.CLEAR;
          } else if(intermediate.getElevatorDirection() == Direction.REVERSE){
            m_state = State.SYSTEM;
          }
          else m_state = State.CLEAR;
        }
        else if (m_state == State.SHOOTER && !intermediate.ShooterBBisTripped()) {
          if (intermediate.getShooterDirection() == Direction.FORWARD) {
            m_state = State.CLEAR;
          }
          else if(intermediate.getShooterDirection() == Direction.REVERSE){
            m_state = State.SYSTEM;
          }
          else m_state = State.CLEAR;
        }
    }
  }

  public State getState() {
    return m_state;
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























