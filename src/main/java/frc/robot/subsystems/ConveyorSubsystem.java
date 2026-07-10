// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ConveyorConstants;

public class ConveyorSubsystem extends SubsystemBase {
  private final TalonFX conveyorMotor;
  private final TalonFXConfiguration conveyorConfig;
  // private final SimpleMotorFeedforward conveyorFeedforward;

  private double conveyorSpeed;
  // private double conveyorFeedforwardOutput;

  /** Creates a new IndexerSub. */
  public ConveyorSubsystem() {
    conveyorMotor = new TalonFX(ConveyorConstants.conveyorMotor_ID);
    conveyorConfig = new TalonFXConfiguration();
    // conveyorFeedforward = new SimpleMotorFeedforward(ConveyorConstants.conveyor_kS, ConveyorConstants.conveyor_kV);

    conveyorConfig.CurrentLimits.SupplyCurrentLimit = Constants.motorMaxCurrent;
    conveyorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    conveyorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    conveyorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    conveyorMotor.getConfigurator().apply(conveyorConfig);
    stop_Wheel();
    conveyorMotor.getDeviceTemp().setUpdateFrequency(4);
    conveyorMotor.getVelocity().setUpdateFrequency(50);
  }

  public void run_Wheel(){
    conveyorSpeed = 6;
  }

  public void run_Wheel_Auto(){
    conveyorSpeed = 8.4;
  }

  public void poop_Wheel(){
    conveyorSpeed = -6;
  }

  public void intake_Wheel(){
    conveyorSpeed = 2;
  }

  public void stop_Wheel(){
    conveyorSpeed = 0;
  }

  public double getMotorTemp(){
    return conveyorMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getMotorVelocity(){
    return conveyorMotor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // conveyorFeedforwardOutput = conveyorFeedforward.calculate(conveyorSpeed);

    conveyorMotor.setVoltage(conveyorSpeed);

    // SmartDashboard.putNumber("Conveyor/MotorVelocity", getMotorVelocity());
    // SmartDashboard.putNumber("ConveyorFeedforwardOutput", conveyorFeedforwardOutput);
  }
}
