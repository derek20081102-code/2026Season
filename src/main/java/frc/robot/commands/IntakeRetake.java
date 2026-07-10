// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRetake extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ConveyorSubsystem m_ConveyorSubsystem;

  /** Creates a new IntakeRetake. */
  public IntakeRetake(IntakeSubsystem m_IntakeSubsystem, ConveyorSubsystem m_ConveyorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.m_ConveyorSubsystem = m_ConveyorSubsystem;
    addRequirements(m_IntakeSubsystem, m_ConveyorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ConveyorSubsystem.run_Wheel();
    m_IntakeSubsystem.primitive_Pivot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.intake_Wheel();
    m_ConveyorSubsystem.run_Wheel();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stop_Wheel();
    m_ConveyorSubsystem.stop_Wheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
