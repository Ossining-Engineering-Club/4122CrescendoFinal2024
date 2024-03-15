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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ClimberMoveTo;
import frc.robot.commands.GoToNote;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.TurretAlign;
import frc.robot.commands.ShooterCommands.ShooterManualAngleControl;
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

public class RobotContainer {
  private final Limelight m_shooterLimelight = new Limelight("limelight");
  private final Limelight m_noteLimelight = new Limelight("limelight-note");
  private final Drivetrain m_robotDrive = new Drivetrain(60, m_shooterLimelight);
  CommandXboxController m_driverController = new CommandXboxController(0);
  CommandXboxController m_secondaryController = new CommandXboxController(1);

  private DigitalInput m_autoSwitch0 = new DigitalInput(constants.kAutoSwitch0Pin);
  private DigitalInput m_autoSwitch1 = new DigitalInput(constants.kAutoSwitch1Pin);
  private DigitalInput m_autoSwitch2 = new DigitalInput(constants.kAutoSwitch2Pin);
  private DigitalInput m_autoSwitch3 = new DigitalInput(constants.kAutoSwitch3Pin);

  private Leds m_led = new Leds(constants.kPWMLedPin);

  private Intake m_intake = new Intake(constants.kIntakeMotorID, constants.kIntakeBreakbeamPin);
  private ShooterFeeder m_shooterFeeder = new ShooterFeeder(constants.kShooterFeederID, constants.kShooterBreakbeamPin);
  private ShooterFlywheels m_shooterFlywheels = new ShooterFlywheels(constants.kShooterFlywheel1ID, constants.kShooterFlywheel2ID);
  private ShooterPivot m_shooterPivot = new ShooterPivot(constants.kShooterPivotID,
                                                          constants.kShooterAngleEncoderChannelA,
                                                          constants.kShooterAngleEncoderChannelB,
                                                          constants.kStartAngle,
                                                          false);
  // private Climber m_climber;

  // public State m_state;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("GoToNote", new GoToNote(m_robotDrive, m_noteLimelight, m_intake, m_led));
    NamedCommands.registerCommand("Shoot", new ShootAuto(m_shooterFlywheels, m_shooterFeeder, m_led));
    NamedCommands.registerCommand("TurretAlign", new TurretAlign(m_robotDrive, m_shooterPivot, m_shooterLimelight));
    NamedCommands.registerCommand("IntakeNoteToShooter", new IntakeNoteToShooter(m_intake, m_shooterFeeder, m_led));

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
                    JoystickMath.convert(m_driverController.getLeftY(), 2, 0.0, 1),
                    JoystickMath.convert(m_driverController.getLeftX(), 2, 0.0, 1),
                    JoystickMath.convert(m_driverController.getRightX(), 2, 0.0, 1),
                    true,
                    true),
            m_robotDrive));

  }

  private void configureButtonBindings() {

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
    m_secondaryController.rightBumper().onTrue(new Shoot(m_shooterFlywheels, m_shooterFeeder, m_led));

    // amp shot
    m_secondaryController.leftBumper().onTrue(new AmpShoot(m_shooterFeeder, m_led));
    
    // forwards/reverse
    // (new OECTrigger(() -> true))
    //   .everyTimeItsTrue(
    //     new ConditionalCommand(
    //       Commands.runOnce(() -> {m_shooterFeeder.setReverse(true); m_intake.setReverse(true);}),
    //       Commands.runOnce(() -> {m_shooterFeeder.setReverse(false); m_intake.setReverse(false);}),
    //       m_secondaryController.button(constants.kForwardsOrReverseButton)::getAsBoolean));
 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
    public Command getAutonomousCommand() {
      //return new PathPlannerAuto("Pos2-P-A-B-C");

      int val = 0b0;
      val += (m_autoSwitch0.get() ? 1 : 0) << 0;
      val += (m_autoSwitch1.get() ? 1 : 0) << 1;
      val += (m_autoSwitch2.get() ? 1 : 0) << 2;
      val += (m_autoSwitch3.get() ? 1 : 0) << 3;

      if (val == 0b0000) return Commands.runOnce(() -> {});
      else if (val == 0b0001) return Commands.runOnce(() -> {});
      else if (val == 0b0010) return Commands.runOnce(() -> {});
      else if (val == 0b0011) return Commands.runOnce(() -> {});
      else if (val == 0b0100) return Commands.runOnce(() -> {});
      else if (val == 0b0101) return Commands.runOnce(() -> {});
      else if (val == 0b0111) return Commands.runOnce(() -> {});
      else if (val == 0b1000) return Commands.runOnce(() -> {});
      else return Commands.runOnce(() -> {});
      //return new PathPlannerAuto("Pos2-P-A-B-C");
    }

    public void enabledInit() {
      m_shooterPivot.m_Angle.set(0);
      m_shooterFlywheels.m_Shooter1.set(0.0);
      m_shooterFlywheels.m_Shooter2.set(0.0);
      m_shooterFeeder.disableFeeder();
      m_intake.stop();

      // Start robot with red LEDS
      m_led.setRed(); 
    }
    
}























