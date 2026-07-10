// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootFuel extends Command {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final ConveyorSubsystem m_ConveyorSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;

  private final DoubleSupplier xSpeedFunc;
  private final DoubleSupplier ySpeedFunc;

  private final BooleanSupplier ifPushFunc;

  private double xSpeed;
  private double ySpeed;

  private boolean ifPush = false;
  private boolean ifIndexer = false;

  /** Creates a new Shoot. */
  public ShootFuel(SwerveSubsystem m_SwerveSubsystem, ConveyorSubsystem m_ConveyorSubsystem, ShooterSubsystem m_ShooterSubsystem, IntakeSubsystem m_IntakeSubsystem, DoubleSupplier xSpeedFunc, DoubleSupplier ySpeedFunc, BooleanSupplier ifPushFunc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_ConveyorSubsystem = m_ConveyorSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.ifPushFunc = ifPushFunc;

    addRequirements(m_SwerveSubsystem, m_ConveyorSubsystem, m_ShooterSubsystem, m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!m_SwerveSubsystem.outOfAllianceZone()){
      LEDConstants.shootHUB = true;
      LEDConstants.shootHome = false;
      LEDConstants.ledFlag = true;
    }else{
      LEDConstants.shootHUB = false;
      LEDConstants.shootHome = true;
      LEDConstants.ledFlag = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ifPush = ifPushFunc.getAsBoolean();
    if (m_ShooterSubsystem.readyShoot_Indexer()) {
      ifIndexer = true;
    }

    if (!m_SwerveSubsystem.outOfAllianceZone()) {
      m_SwerveSubsystem.aimHubRotation();
      m_ShooterSubsystem.shoot_Shooter();
      m_ShooterSubsystem.setAngleOfElevation();
      m_IntakeSubsystem.slowIntake_Wheel();
      if (ifPush) {
        m_IntakeSubsystem.pushFuel_Pivot();
      }else{
        m_IntakeSubsystem.intakeFuel_Pivot();
      }
      if (m_ShooterSubsystem.elevationArriveSetpoint() && (m_ShooterSubsystem.readyShoot_Shooter_Left() || m_ShooterSubsystem.readyShoot_Shooter_Right())) {
        m_ShooterSubsystem.pull_Indexer();
        if(ifIndexer){
          m_ConveyorSubsystem.run_Wheel();
        }else{
          m_ConveyorSubsystem.stop_Wheel();
        }
      }
    }else {
      xSpeed = -xSpeedFunc.getAsDouble() * 0.3;
      ySpeed = -ySpeedFunc.getAsDouble() * 0.3;
      m_SwerveSubsystem.aimHomeRotation(xSpeed, ySpeed);
      m_ShooterSubsystem.shootHome_Shooter();
      m_ShooterSubsystem.setAngleOfElevationToHome();
      m_IntakeSubsystem.intake_Wheel();
      if (ifPush) {
        m_IntakeSubsystem.pushFuel_Pivot();
      }else{
        m_IntakeSubsystem.intakeFuel_Pivot();
      }
      if (m_ShooterSubsystem.elevationArriveSetpoint() && (m_ShooterSubsystem.readyShoot_Shooter_Left() || m_ShooterSubsystem.readyShoot_Shooter_Right())) {
        m_ShooterSubsystem.pull_Indexer();
        if(ifIndexer){
          m_ConveyorSubsystem.run_Wheel();
        }else {
          m_ConveyorSubsystem.stop_Wheel();
        }
      } 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.stop();
    m_ConveyorSubsystem.stop_Wheel();
    m_ShooterSubsystem.stop_Indexer();
    m_ShooterSubsystem.stop_Shooter();
    m_ShooterSubsystem.primitiveAngleOfElevation();
    m_IntakeSubsystem.stop_Wheel();
    LEDConstants.shootHUB = false;
    LEDConstants.shootHome = false;
    LEDConstants.ledFlag = true;
    // m_IntakeSubsystem.outMotion();
    // m_IntakeSubsystem.intakeFuel_Pivot();
    m_SwerveSubsystem.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
