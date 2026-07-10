// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShoot extends Command {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private final ConveyorSubsystem m_ConveyorSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private boolean ifIndexer;

  /** Creates a new shoot. */
  public AutoShoot(SwerveSubsystem m_SwerveSubsystem, ShooterSubsystem m_ShooterSubsystem, ConveyorSubsystem m_ConveyorSubsystem, IntakeSubsystem m_IntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = m_SwerveSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    this.m_ConveyorSubsystem = m_ConveyorSubsystem;
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    ifIndexer = false;
    addRequirements(m_SwerveSubsystem, m_ShooterSubsystem, m_ConveyorSubsystem, m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LEDConstants.shootHUB = true;
    LEDConstants.ledFlag = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(m_ShooterSubsystem.readyShoot_Indexer()){
        ifIndexer = true;
      }
      m_ShooterSubsystem.shoot_Shooter();
      m_ShooterSubsystem.setAngleOfElevation();
      m_IntakeSubsystem.slowIntake_Wheel();
      m_SwerveSubsystem.aimHubRotation_Auto();
      if (m_SwerveSubsystem .arriveAimHubRotation() && m_ShooterSubsystem.elevationArriveSetpoint() && (m_ShooterSubsystem.readyShoot_Shooter_Left() || m_ShooterSubsystem.readyShoot_Shooter_Right())) { 
        m_ShooterSubsystem.pull_Indexer();
        if(ifIndexer){
          m_ConveyorSubsystem.run_Wheel_Auto();
        }
        else{
          m_ConveyorSubsystem.stop_Wheel();
        } 
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stop_Indexer();
    m_ShooterSubsystem.stop_Shooter();
    m_ConveyorSubsystem.stop_Wheel();
    m_IntakeSubsystem.stop_Wheel();
    m_SwerveSubsystem.drive(0, 0, 0, false);
    LEDConstants.shootHUB = false;
    LEDConstants.ledFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
