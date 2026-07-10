// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.IntakeRetake;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.PoopFuel;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.AutoCommand.AutoAimHub;
import frc.robot.commands.AutoCommand.AutoIntakeOut;
import frc.robot.commands.AutoCommand.AutoIntakeRun;
import frc.robot.commands.AutoCommand.AutoPushShooter;
import frc.robot.commands.AutoCommand.AutoShoot;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem(m_VisionSubsystem);
  // private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final ConveyorSubsystem m_ConveyorSubsystem = new ConveyorSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(m_VisionSubsystem);
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("AimHub", new AutoAimHub(m_SwerveSubsystem));
    NamedCommands.registerCommand("WaitForOutpost", Commands.run(() -> {m_SwerveSubsystem.stop();}).withTimeout(1.5));
    NamedCommands.registerCommand("shoot", new AutoShoot(m_SwerveSubsystem, m_ShooterSubsystem, m_ConveyorSubsystem, m_IntakeSubsystem).withTimeout(4));
    NamedCommands.registerCommand("PushShoot", new AutoPushShooter(m_SwerveSubsystem, m_IntakeSubsystem, m_ShooterSubsystem, m_ConveyorSubsystem).withTimeout(3));
    NamedCommands.registerCommand("StopSwerve", Commands.run(() -> {m_SwerveSubsystem.stop();}).withTimeout(0.5));
    NamedCommands.registerCommand("IntakeStop", Commands.runOnce(() -> {m_IntakeSubsystem.intakeShouldStop();}));
    NamedCommands.registerCommand("IntakeStart", Commands.runOnce(() -> {m_IntakeSubsystem.intakeShouldStart();}));
    // NamedCommands.registerCommand("NeutralIntake", new AutoIntake(m_IntakeSubsystem).withTimeout(5));
    NamedCommands.registerCommand("ShooterTurn", Commands.runOnce(() -> {m_ShooterSubsystem.shooterTurn();}));
    NamedCommands.registerCommand("IntakeOut", new AutoIntakeOut(m_IntakeSubsystem));
    NamedCommands.registerCommand("IntakeRun", new AutoIntakeRun(m_IntakeSubsystem).until(() -> m_IntakeSubsystem.intakeMode()));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", autoChooser);
    configureBindings();
}

  public void xboxStop(){
    Commands.run(() -> {driverController.setRumble(RumbleType.kBothRumble, 1);}).withTimeout(0.3);
  }
  public void xboxStart(){
    Commands.run(() -> {driverController.setRumble(RumbleType.kBothRumble, 1);}).withTimeout(0.5);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    DoubleSupplier xSpeedFunc = ()-> driverController.getRawAxis(1);
    DoubleSupplier ySpeedFunc = ()-> driverController.getRawAxis(0);
    DoubleSupplier zSpeedFunc = ()-> driverController.getRawAxis(4);

    m_SwerveSubsystem.setDefaultCommand(new ManualDrive(m_SwerveSubsystem, xSpeedFunc, ySpeedFunc, zSpeedFunc));
    driverController.x().whileTrue(Commands.runOnce(() -> m_SwerveSubsystem.resetGyro()));

    BooleanSupplier ifPushFunc = ()-> driverController.getHID().getYButton();

    driverController.leftTrigger(0.5).whileTrue(new IntakeFuel(m_IntakeSubsystem, m_ConveyorSubsystem, m_ShooterSubsystem));
    driverController.rightTrigger(0.5).whileTrue(new ShootFuel(m_SwerveSubsystem, m_ConveyorSubsystem, m_ShooterSubsystem, m_IntakeSubsystem, xSpeedFunc, ySpeedFunc, ifPushFunc));
    driverController.a().whileTrue(new IntakeRetake(m_IntakeSubsystem, m_ConveyorSubsystem));
    driverController.b().whileTrue(new PoopFuel(m_IntakeSubsystem, m_ConveyorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    // return null;
  }
}
