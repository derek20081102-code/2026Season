// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.ops.ConvertMatrixData;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFuel extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ConveyorSubsystem m_ConveyorSubsystem;
  private final ShooterSubsystem m_ShooterSubsystem;
  private boolean ledHasChange;

  /** Creates a new IntakeFuel. */
  public IntakeFuel(IntakeSubsystem m_IntakeSubsystem, ConveyorSubsystem m_ConveyorSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ConveyorSubsystem = m_ConveyorSubsystem;
    this.m_ShooterSubsystem = m_ShooterSubsystem;
    addRequirements(m_IntakeSubsystem, m_ConveyorSubsystem, m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeFuel_Pivot();
    ledHasChange = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_IntakeSubsystem.gearArriveSetpoint()) {
      m_IntakeSubsystem.intake_Wheel();
      m_ConveyorSubsystem.intake_Wheel();
      m_ShooterSubsystem.intake_Indexer();
      if(!ledHasChange){
        LEDConstants.intake = true;
        LEDConstants.ledFlag = true;
        ledHasChange = true;
      }
    // }
    // m_IntakeSubsystem.intake_Wheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop_Wheel();
    m_ConveyorSubsystem.stop_Wheel();
    m_ShooterSubsystem.stop_Indexer();
    LEDConstants.intake = false;
    LEDConstants.ledFlag = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
