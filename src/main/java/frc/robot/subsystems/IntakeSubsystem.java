// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX gearMotor;
  private final TalonFXConfiguration gearConfig;
  private final MotionMagicVoltage gearRequest;
  private final MotionMagicConfigs gearMotionMagicConfigs;

  private double gearSetpoint;

  private final TalonFX wheelMotor;
  private final TalonFXConfiguration wheelConfig;

  private double wheelSpeed;
  private double wheelFeedforwardOutput;
  private boolean intakeSoHigh;
  private boolean intakeStop;

  /** Creates a new IntakeSubSystem. */
  public IntakeSubsystem() {
    gearMotor = new TalonFX(IntakeConstants.gearMotor_ID);
    gearConfig = new TalonFXConfiguration();
    gearMotionMagicConfigs = new MotionMagicConfigs();

    gearConfig.CurrentLimits.SupplyCurrentLimit = Constants.motorMaxCurrent;
    gearConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    gearConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    gearConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    gearConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    gearConfig.Slot1.kP = IntakeConstants.gear_kP;
    gearConfig.Slot1.kI = IntakeConstants.gear_kI;
    gearConfig.Slot1.kD = IntakeConstants.gear_kD;
    gearConfig.Slot1.kG = IntakeConstants.gear_kG;
    gearConfig.Slot1.kS = IntakeConstants.gear_kS;
    gearConfig.Slot1.kV = IntakeConstants.gear_kV;
    gearMotor.getConfigurator().apply(gearConfig);

    gearMotionMagicConfigs.MotionMagicCruiseVelocity = 75;
    gearMotionMagicConfigs.MotionMagicAcceleration = 230;
    gearMotionMagicConfigs.MotionMagicJerk = 2400;
    gearMotor.getConfigurator().apply(gearMotionMagicConfigs);

    wheelMotor = new TalonFX(IntakeConstants.wheelMotor_ID);
    wheelConfig = new TalonFXConfiguration();

    wheelConfig.CurrentLimits.SupplyCurrentLimit = Constants.motorMaxCurrent;
    wheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    wheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wheelMotor.getConfigurator().apply(wheelConfig);

    resetGearPosition();
    gearRequest = new MotionMagicVoltage(gearSetpoint);
    primitive_Pivot();
    intakeShouldStart();

    gearMotor.getDeviceTemp().setUpdateFrequency(4);
    gearMotor.getPosition().setUpdateFrequency(50);
  }
  
  public void primitive_Pivot(){
    // gearSetpoint = IntakeConstants.primitivePosition;
    gearSetpoint = 0;
  }

  public void intakeFuel_Pivot(){
    gearSetpoint = IntakeConstants.intakeFuelPosition;
  }

  public void pushFuel_Pivot(){
    gearSetpoint = IntakeConstants.pushFuelPosition;
  }

  public void resetGearPosition(){
    gearMotor.setPosition(0);
  }

  public void intake_Wheel(){
    // wheelSpeed = IntakeConstants.wheelTargetSpeed;
    wheelSpeed = 8.4 ;
  } 

  public void intake_Wheel_Auto(){
    wheelSpeed = 11.5;
  }

  public void slowIntake_Wheel(){
    wheelSpeed = 6;
  }

  public void poopIntake_Wheel(){
    wheelSpeed = -9;
  }

  public void stop_Wheel(){
    wheelSpeed = 0;
  }

  public boolean gearArriveSetpoint(){
    return Math.abs(gearSetpoint - getGearPosition_Motor()) <= IntakeConstants.gearAcceptError;
  }

  public boolean gearArriveSetpoint_Auto(){
    return Math.abs(gearSetpoint - getGearPosition_Motor()) <= 0.5;
  }

  public double getGearTemp(){
    return gearMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getGearPosition_Motor(){
    return gearMotor.getPosition().getValueAsDouble();
  }

  public double getWheelTemp(){
    return wheelMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getWheelVelocity(){
    return wheelMotor.getVelocity().getValueAsDouble();
  }

  public void intakeShouldStop(){
    intakeStop = true;
  }

  public void intakeShouldStart(){
    intakeStop = false;
  }

  public boolean intakeMode(){
    return intakeStop;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    gearMotor.setControl(gearRequest.withPosition(gearSetpoint).withSlot(1));    

    intakeSoHigh = getWheelTemp() >= 70;

    if(intakeSoHigh)  wheelMotor.stopMotor();
    else  wheelMotor.setVoltage(wheelSpeed);

    SmartDashboard.putBoolean("Intake/Gear/ArriveSetpoint", gearArriveSetpoint());
    // SmartDashboard.putNumber("Intake/Gear/Temp", getGearTemp());
    // SmartDashboard.putNumber("Intake/Gear/Position/Motor", getGearPosition_Motor());
    // SmartDashboard.putNumber("Intake/Gear/Setpoint", gearSetpoint);
    
    SmartDashboard.putBoolean("Intake/Wheel/WheelSoHigh", intakeSoHigh);
    // SmartDashboard.putNumber(" ntake/Wheel/Temp", getWheelTemp());
    // SmartDashboard.putNumber("Intake/Wheel/Velocity", getWheelVelocity());
    // SmartDashboard.putNumber("Intake/Wheel/FeedforwardOutput", wheelFeedforwardOutput);
    // SmartDashboard.putBoolean("Intake/Mode", intakeMode());
  }
}
