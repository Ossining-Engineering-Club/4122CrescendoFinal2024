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

import org.w3c.dom.NamedNodeMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ClimberMoveTo;
import frc.robot.commands.GoToNote;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TurretAlign;
import frc.robot.commands.ShooterCommands.ShooterManualAngleControl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Breakbeam;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.OECTrigger;
import frc.robot.constants.State;
import frc.robot.constants.Direction;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.ShooterFlywheels;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Leds;
import frc.robot.JoystickMath;
import frc.robot.commands.AmpShoot;
import frc.robot.commands.ClimberManualControl;
import frc.robot.commands.GoToAndIntakeNote;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.IntakeNoteToShooter;
import frc.robot.commands.ShooterCommands.AngleShooter;
import frc.robot.commands.GoToNoteAuto;
import frc.robot.commands.ReverseNote;

public class RobotContainer {
  private final Limelight m_shooterLimelight = new Limelight("limelight");
  private final Limelight m_noteLimelight = new Limelight("limelight-note");
  private final Drivetrain m_robotDrive = new Drivetrain(60, m_shooterLimelight);
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandXboxController m_secondaryController = new CommandXboxController(1);

  // private DigitalInput m_autoSwitch0 = new DigitalInput(constants.kAutoSwitch0Pin);
  // private DigitalInput m_autoSwitch1 = new DigitalInput(constants.kAutoSwitch1Pin);
  // private DigitalInput m_autoSwitch2 = new DigitalInput(constants.kAutoSwitch2Pin);
  // private DigitalInput m_autoSwitch3 = new DigitalInput(constants.kAutoSwitch3Pin);

  private Leds m_led = new Leds(constants.kPWMLedPin);

  private Intake m_intake = new Intake(constants.kIntakeMotorID, constants.kIntakeBreakbeamPin);
  private ShooterFeeder m_shooterFeeder = new ShooterFeeder(constants.kShooterFeederID, constants.kShooterBreakbeamPin);
  private ShooterFlywheels m_shooterFlywheels = new ShooterFlywheels(constants.kShooterFlywheel1ID, constants.kShooterFlywheel2ID, m_led);
  private ShooterPivot m_shooterPivot = new ShooterPivot(constants.kShooterPivotID,
                                                          constants.kStartAngle,
                                                          true/*,
                                                          constants.kShooterLimitSwitchPin*/);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();
  // private Climber m_climber;

  // public State m_state;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("GoToNote", new GoToNoteAuto(m_robotDrive, m_intake));
    NamedCommands.registerCommand("Shoot", new ShootAuto(m_shooterFlywheels, m_shooterFeeder, m_led));
    NamedCommands.registerCommand("TurretAlign", new TurretAlign(m_robotDrive, m_shooterPivot, m_shooterLimelight));
    NamedCommands.registerCommand("IntakeNoteToShooter", new IntakeNoteToShooter(m_intake, m_shooterFeeder, m_led));
    NamedCommands.registerCommand("Pos1Or3AngleShooter", new AngleShooter(m_shooterPivot, constants.kPos1Or3ShooterAngle));
    NamedCommands.registerCommand("StowShooter", new AngleShooter(m_shooterPivot, constants.kStartAngle));

    m_shooterLimelight.setPipeline(0);
    m_noteLimelight.setPipeline(0);

    // Configure the button bindings
    configureButtonBindings();
    configureAutos();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> 
                m_robotDrive.Drive(
                    JoystickMath.convert(m_driverController.getLeftY(), 2, 0.0, 1),
                    JoystickMath.convert(m_driverController.getLeftX(), 2, 0.0, 1),
                    JoystickMath.convert(m_driverController.getRightX(), 2, 0.0, 1),
                    true,
                    true),
            m_robotDrive));

  }

  private void configureButtonBindings() {

    m_driverController.a().onTrue(
      Commands.runOnce(() -> m_robotDrive.resetPose(new Pose2d(0, 0, new Rotation2d(0)))));

    // manual angle control
    m_shooterPivot.setDefaultCommand(new ShooterManualAngleControl(
                                           m_shooterPivot,
                                           () -> MathUtil.applyDeadband(m_secondaryController.getRightY(), 0.1)));
    
    // cancel everything
    m_secondaryController.povDown().onTrue(
      Commands.runOnce(() -> {enabledInit();},
      m_robotDrive,
      m_intake,
      m_shooterFeeder,
      m_shooterFlywheels,
      m_shooterPivot));

    // go to note
    // m_secondaryController.a()
    //   .onTrue(
    //     new GoToNote(m_robotDrive,m_noteLimelight,m_intake,m_led));

    // m_secondaryController.a()
    //   .onTrue(
    //     new GoToNoteAuto(m_robotDrive,m_intake));

    m_secondaryController.a()
      .onTrue(
        new IntakeNoteToShooter(m_intake, m_shooterFeeder, m_led));

    // set shooter to subwoofer angle
    m_secondaryController.b().onTrue(new AngleShooter(m_shooterPivot, constants.kShooterSubwooferAngle));
    m_secondaryController.b().onTrue(Commands.runOnce(() -> m_shooterFlywheels.start(), m_shooterFlywheels));

    // set shooter to podium angle
    m_secondaryController.x().onTrue(new AngleShooter(m_shooterPivot, constants.kShooterPodiumAngle));
    m_secondaryController.x().onTrue(Commands.runOnce(() -> m_shooterFlywheels.start(), m_shooterFlywheels));

    // set shooter to amp angle
    m_secondaryController.y().onTrue(new AngleShooter(m_shooterPivot, constants.kShooterAmpAngle));

    // speaker shot
    m_secondaryController.rightBumper().onTrue(new Shoot(m_shooterFlywheels, m_shooterFeeder, m_led)
                                      .andThen(new AngleShooter(m_shooterPivot, constants.kStartAngle)));

    // amp shot
    m_secondaryController.leftBumper().whileTrue(new AmpShoot(m_shooterFeeder, m_led));
    m_secondaryController.leftBumper().onFalse(Commands.runOnce(() -> m_shooterFeeder.disableFeeder(), m_shooterFeeder));
    m_secondaryController.leftBumper().onFalse(Commands.runOnce(() -> m_shooterFeeder.setReverse(false), m_shooterFeeder));

    m_secondaryController.rightTrigger(0.9).whileTrue(new ReverseNote(m_intake, m_shooterFeeder, m_led));

    // manual feeder
    m_secondaryController.leftTrigger(0.9).onTrue(Commands.runOnce(() -> m_shooterFeeder.enableFeeder()));
    m_secondaryController.leftTrigger(0.9).onFalse(Commands.runOnce(() -> m_shooterFeeder.disableFeeder()));
  }

  public void configureAutos() {
    m_autoChooser.setDefaultOption("Do Nothing", new PathPlannerAuto("Forward-Nothing"));
    m_autoChooser.addOption("1P", new PathPlannerAuto("Pos1-P"));
    m_autoChooser.addOption("2P", new PathPlannerAuto("Pos2-P"));
    m_autoChooser.addOption("3P", new PathPlannerAuto("Pos3-P"));
    m_autoChooser.addOption("2PB", new PathPlannerAuto("Pos2-P-B"));
    m_autoChooser.addOption("2PABC", new PathPlannerAuto("Pos2-P-A-B-C"));
    m_autoChooser.addOption("3PHG", new PathPlannerAuto("Pos3-P-H-G"));

    SmartDashboard.putData(m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
      return m_autoChooser.getSelected();
      //return new PathPlannerAuto("Pos2-P-A-B-C");

      // int val = 0b0;
      // val += (m_autoSwitch0.get() ? 1 : 0) << 0;
      // val += (m_autoSwitch1.get() ? 1 : 0) << 1;
      // val += (m_autoSwitch2.get() ? 1 : 0) << 2;
      // val += (m_autoSwitch3.get() ? 1 : 0) << 3;
      // int val = (m_autoSwitch0.get() ? 1 : 0)
      //       + 2*(m_autoSwitch1.get() ? 1 : 0)
      //       + 4*(m_autoSwitch2.get() ? 1 : 0)
      //       + 8*(m_autoSwitch3.get() ? 1 : 0);

      // switch (val) {
      //   case 0:
      //     return new PathPlannerAuto("Forward-Nothing");
      //   case 1:
      //     return new PathPlannerAuto("Pos1-P");
      //   case 2:
      //     return new PathPlannerAuto("Pos2-P");
      //   case 3:
      //     return new PathPlannerAuto("Pos3-P");
      //   case 4:
      //     return new PathPlannerAuto("Pos2-P-B");
      //   case 15:
      //     return new PathPlannerAuto("Pos2-P-A-B-C");
      //   default:
      //     return new PathPlannerAuto("Forward-Nothing");
      // }

      // if (val == 0b0000) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 0");});
      // else if (val == 0b0001) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 1");});
      // else if (val == 0b0010) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 2");});
      // else if (val == 0b0011) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 3");});
      // else if (val == 0b0100) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 4");});
      // else if (val == 0b0101) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 5");});
      // else if (val == 0b0111) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 6");});
      // else if (val == 0b1000) return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto 7");});
      // else return Commands.runOnce(() -> {SmartDashboard.putString("auto", "auto default");});

      // if (m_autoSwitch3.get() && m_autoSwitch2.get() && m_autoSwitch1.get() && m_autoSwitch0.get()) {
      //   return new PathPlannerAuto("Pos2-P-A-B-C");
      // }
      // else {
      //   return new PathPlannerAuto("Forward-Nothing");
      // }

      //return new PathPlannerAuto("Pos2-P-A-B-C");
    }

    public void enabledInit() {
      m_shooterPivot.m_Angle.set(0);
      m_shooterFlywheels.stop();
      m_shooterFeeder.disableFeeder();
      m_intake.stop();
      m_shooterFeeder.setReverse(false);

      // Start robot with red LEDS
      m_led.setRed(); 
    }

    public void periodic() {
      // int val = (m_autoSwitch0.get() ? 1 : 0)
      //       + 2*(m_autoSwitch1.get() ? 1 : 0)
      //       + 4*(m_autoSwitch2.get() ? 1 : 0)
      //       + 8*(m_autoSwitch3.get() ? 1 : 0);
      // SmartDashboard.putNumber("auto val", val);
      // SmartDashboard.putBoolean("auto switch 0", m_autoSwitch0.get());
      // SmartDashboard.putBoolean("auto switch 1", m_autoSwitch1.get());
      // SmartDashboard.putBoolean("auto switch 2", m_autoSwitch2.get());
      // SmartDashboard.putBoolean("auto switch 3", m_autoSwitch3.get());
    }
    
}
