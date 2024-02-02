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
import frc.robot.subsystems.Breakbeam;
import frc.robot.constants.State;
import frc.robot.constants.Direction;
import frc.robot.subsystems.Intermediate;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_robotDrive = new Drivetrain(13);
  private final Limelight m_aprilTagLimelight = new Limelight("limelight");
  private final Limelight m_noteLimelight = new Limelight("limelight");
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  CommandXboxController m_driverController = new CommandXboxController(0);

  private final Breakbeam testBreakbeam = new Breakbeam(3);
  private Intermediate intermediate;
  private Intake intake;
  public State m_state;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //NamedCommands.registerCommand("GoToNote", new GoToNote(m_robotDrive, m_noteLimelight));
    m_state = State.CLEAR;
    m_aprilTagLimelight.setPipeline(0);
    m_noteLimelight.setPipeline(0);
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> 
                m_robotDrive.Drive(
                    0.4*MathUtil.applyDeadband(-m_driverController.getLeftY(), constants.kControllerDeadband)*constants.kMaxSpeed,
                    0.4*MathUtil.applyDeadband(-m_driverController.getLeftX(),constants.kControllerDeadband)*constants.kMaxSpeed,
                    0.4*MathUtil.applyDeadband(-m_driverController.getRightX(),constants.kControllerDeadband)*constants.kMaxAngularSpeed,
                    true),
            m_robotDrive));    
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

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
        }
        else if (m_state == State.ELEVATOR && !intermediate.ElevatorBBisTripped()) {
          if (intermediate.getElevatorDirection() == Direction.FORWARD) {
            m_state = State.CLEAR;
          } else if(intermediate.getElevatorDirection() == Direction.REVERSE){
            m_state = State.SYSTEM;
          }  
        }
        else if (m_state == State.SHOOTER && !intermediate.ShooterBBisTripped()) {
          if (intermediate.getShooterDirection() == Direction.FORWARD) {
            m_state = State.CLEAR;
          }
          else if(intermediate.getShooterDirection() == Direction.REVERSE){
            m_state = State.SYSTEM;
          }
        }
    }
  }
  
  private void configureButtonBindings() {
    m_driverController.a().onTrue(new TurretMode(
      m_robotDrive, 
      m_aprilTagLimelight, 
      -8.308975,
      1.442593, 
      () -> -m_driverController.getLeftY(), 
      () -> -m_driverController.getLeftX(), 
      () -> -m_driverController.getRightX()));
    //m_driverController.b().onTrue(new GoToNote(m_robotDrive, m_noteLimelight));
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























