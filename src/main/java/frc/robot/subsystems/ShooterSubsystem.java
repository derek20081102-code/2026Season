// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterListConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final VisionSubsystem m_VisionSubsystem;

  private final TalonFX indexerMotor;
  private final TalonFXConfiguration indexerConfig;
  private final SimpleMotorFeedforward indexerFeedforward;

  private double indexerFeedforwardOutput;
  private double indexerSpeed;

  private final SparkFlex shooterLeftMotor;
  private final SparkFlex shooterRightMotor;
  private final SparkFlexConfig shooterLeftConfig;
  private final SparkFlexConfig shooterRightConfig;
  private final RelativeEncoder shooterLeftEncoder;
  private final RelativeEncoder shooterRightEncoder;
  private final SimpleMotorFeedforward shooterLeftFeedforward;
  private final SimpleMotorFeedforward shooterRightFeedforward;

  private double shooterLeftFeedforwardOutput;
  private double shooterRightFeedforwardOutput;
  private double shooterLeftSpeed;
  private double shooterRightSpeed;

  private final TalonFX elevationMotor;
  private final TalonFXConfiguration elevationMotorConfig;
  private final CANcoder elevationCANcoder;
  private final CANcoderConfiguration elevationCANcoderConfig;
  private final MotionMagicVoltage elevationRequest;
  private final MotionMagicConfigs elevationMotionMagicConfigs;

  private double elevationSetpoint;

  /** Creates a new ShooterSub. */
  public ShooterSubsystem(VisionSubsystem m_VisionSubsystem) {
    this.m_VisionSubsystem = m_VisionSubsystem;

    indexerMotor = new TalonFX(ShooterConstants.indexerMotor_ID);
    indexerConfig = new TalonFXConfiguration();
    indexerFeedforward = new SimpleMotorFeedforward(ShooterConstants.indexer_kS, ShooterConstants.indexer_kV);

    indexerConfig.CurrentLimits.SupplyCurrentLimit = Constants.motorMaxCurrent;
    indexerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexerMotor.getConfigurator().apply(indexerConfig);

    shooterLeftMotor = new SparkFlex(ShooterConstants.shooterLeftMotor_ID, MotorType.kBrushless);
    shooterRightMotor = new SparkFlex(ShooterConstants.shooterRightMotor_ID, MotorType.kBrushless);
    shooterLeftConfig = new SparkFlexConfig();
    shooterRightConfig = new SparkFlexConfig();
    shooterLeftEncoder = shooterLeftMotor.getEncoder();
    shooterRightEncoder = shooterRightMotor.getEncoder();
    shooterLeftFeedforward = new SimpleMotorFeedforward(ShooterConstants.shooterLeft_kS, ShooterConstants.shooterLeft_kV);
    shooterRightFeedforward = new SimpleMotorFeedforward(ShooterConstants.shooterRight_kS, ShooterConstants.shooterRight_kV);

    shooterLeftConfig.idleMode(IdleMode.kCoast).inverted(true).smartCurrentLimit(Constants.motorMaxCurrent);
    shooterRightConfig.idleMode(IdleMode.kCoast).inverted(false).smartCurrentLimit(Constants.motorMaxCurrent);
    shooterLeftMotor.configure(shooterLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    shooterRightMotor.configure(shooterRightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    elevationMotor = new TalonFX(ShooterConstants.elevation_ID);
    elevationMotorConfig = new TalonFXConfiguration();
    elevationCANcoder = new CANcoder(ShooterConstants.elevationCANcoder_ID);
    elevationCANcoderConfig = new CANcoderConfiguration();
    elevationMotionMagicConfigs = new MotionMagicConfigs();

    elevationMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.motorMaxCurrent;
    elevationMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevationMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevationMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    elevationMotorConfig.Slot1.GravityType = GravityTypeValue.Elevator_Static;
    elevationMotorConfig.Slot1.kP = ShooterConstants.elevation_kP;
    elevationMotorConfig.Slot1.kI = ShooterConstants.elevation_kI;
    elevationMotorConfig.Slot1.kD = ShooterConstants.elevation_kD;
    elevationMotorConfig.Slot1.kS = ShooterConstants.elevation_kS;
    elevationMotorConfig.Slot1.kG = ShooterConstants.elevation_kG;
    elevationMotorConfig.Slot1.kV = ShooterConstants.elevation_kV;    
    elevationMotorConfig.Feedback.FeedbackRemoteSensorID = elevationCANcoder.getDeviceID();
    elevationMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elevationMotorConfig.Feedback.SensorToMechanismRatio = 0.0416;
    elevationMotor.getConfigurator().apply(elevationMotorConfig);

    elevationMotionMagicConfigs.MotionMagicCruiseVelocity = 100;
    elevationMotionMagicConfigs.MotionMagicAcceleration = 200;
    elevationMotionMagicConfigs.MotionMagicJerk = 800;
    elevationMotor.getConfigurator().apply(elevationMotionMagicConfigs);

    elevationCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    elevationCANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    elevationCANcoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.elevationOffset;
    elevationCANcoder.getConfigurator().apply(elevationCANcoderConfig);

    resetElevationPosition();

    elevationRequest = new MotionMagicVoltage(elevationSetpoint);

    primitiveAngleOfElevation();
    shooterLeftMotor.setControlFramePeriodMs(50);
    shooterRightMotor.setControlFramePeriodMs(50);
    indexerMotor.getDeviceTemp().setUpdateFrequency(4);
    indexerMotor.getVelocity().setUpdateFrequency(50);
    elevationMotor.getDeviceTemp().setUpdateFrequency(4);
    elevationMotor.getPosition().setUpdateFrequency(50); 
    elevationCANcoder.getPosition().setUpdateFrequency(50);  
  }

  public void pull_Indexer(){
    indexerSpeed = ShooterConstants.indexerTargetSpeed;
  }

  public void intake_Indexer(){
    indexerSpeed = -750;
  }

  public void stop_Indexer(){
    indexerSpeed = 0;
  }

  public void shooterTurn(){
    shooterLeftSpeed = 600;
    shooterRightSpeed = 600;
  }

  public void shoot_Shooter(){
    shooterLeftSpeed = ShooterListConstants.getSpeed_HUB(m_VisionSubsystem.getRobotToHubDistance()) + 180;
    shooterRightSpeed = ShooterListConstants.getSpeed_HUB(m_VisionSubsystem.getRobotToHubDistance()) + 180;
    // shooterLeftSpeed = 1800;
    // shooterRightSpeed = 1800;
  }

  public void shootHome_Shooter(){
    shooterLeftSpeed = ShooterListConstants.getSpeed_Home(m_VisionSubsystem.getRobotToHomeDistance());
    shooterRightSpeed = ShooterListConstants.getSpeed_Home(m_VisionSubsystem.getRobotToHomeDistance());

    // shooterLeftSpeed = 5000;
    // shooterRightSpeed = 5000;
  }

  public void stop_Shooter(){
    shooterLeftSpeed = 0;
    shooterRightSpeed = 0;
  }

  public void primitiveAngleOfElevation(){
    elevationSetpoint = 0;
  }

  public void setAngleOfElevation(){
    elevationSetpoint = ShooterListConstants.getHoodAngle_HUB(m_VisionSubsystem.getRobotToHubDistance()) - 0.5;//查表
  }

  public void setAngleOfElevationToHome(){
    elevationSetpoint = ShooterListConstants.getHoodAngle_Home(m_VisionSubsystem.getRobotToHomeDistance());
    // elevationSetpoint = 25;
  }

  public void set12345(){
    elevationSetpoint = 35;
  }

  public void resetElevationPosition(){
    elevationCANcoder.setPosition(0);
  }

  public boolean readyShoot_Shooter_Left(){
    return Math.abs(shooterLeftSpeed - getShooterLeftVelocity()) <= ShooterConstants.shooterSpeedError;
  }

  public boolean readyShoot_Shooter_Right(){
    return Math.abs(shooterRightSpeed - getShooterRightVelocity()) <=  ShooterConstants.shooterSpeedError;
  }

  public boolean readyShoot_Indexer(){
    return Math.abs(ShooterConstants.indexerTargetSpeed - (getIndexerVelocity() * 60)) <= ShooterConstants.indexerSpeedError;
  }

  public boolean elevationArriveSetpoint(){
    return Math.abs(getElevationPosition_Motor() - elevationSetpoint) <= ShooterConstants.elevationAcceptError;
  }

  public double getIndexerTemp(){
    return indexerMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getIndexerVelocity(){
    return indexerMotor.getVelocity().getValueAsDouble();
  }

  public double getShooterLeftTemp(){
    return shooterLeftMotor.getMotorTemperature();
  }
  
  public double getShooterLeftVelocity(){
    return shooterLeftEncoder.getVelocity();
  }

  public double getShooterRightTemp(){
    return shooterRightMotor.getMotorTemperature();
  }

  public double getShooterRightVelocity(){
    return shooterRightEncoder.getVelocity();
  }

  public double getElevationTemp(){
    return elevationMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getElevationPosition_Absolute(){
    return elevationCANcoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getElevationPosition_Realtive(){
    return elevationMotor.getPosition().getValueAsDouble();
  }

  public double getElevationPosition_Motor(){
    return elevationMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexerFeedforwardOutput = indexerFeedforward.calculate(indexerSpeed / 60);
    shooterLeftFeedforwardOutput = shooterLeftFeedforward.calculate(shooterLeftSpeed / 60);
    shooterRightFeedforwardOutput = shooterRightFeedforward.calculate(shooterRightSpeed / 60); 

    elevationMotor.setControl(elevationRequest.withPosition(elevationSetpoint).withSlot(1));
    indexerMotor.setVoltage(indexerFeedforwardOutput);
    shooterLeftMotor.setVoltage(shooterLeftFeedforwardOutput);
    shooterRightMotor.setVoltage(shooterRightFeedforwardOutput);

    // SmartDashboard.putNumber("Indexer/MotorTemp", getIndexerTemp());
    // SmartDashboard.putNumber("Indexer/MotorVelocity", getIndexerVelocity());
    // SmartDashboard.putNumber("Indexer/FeedforwardOutput", indexerFeedforwardOutput);

    // SmartDashboard.putNumber("Shooter/Left/MotorTemp", getShooterLeftTemp());
    // SmartDashboard.putNumber("Shooter/Left/MotorVelocity", getShooterLeftVelocity());
    // SmartDashboard.putNumber("Shooter/Left/FeedforwardOutput", shooterLeftFeedforwardOutput);
    // SmartDashboard.putNumber("Shooter/Left/LeftSpeedSetpoint", shooterLeftSpeed);
    // SmartDashboard.putBoolean("Shooter/Left/LeftReady", readyShoot_Shooter_Left());

    // SmartDashboard.putNumber("Shooter/Right/MotorTemp", getShooterRightTemp());
    // SmartDashboard.putNumber("Shooter/Right/MotorVelocity", getShooterRightVelocity());
    // SmartDashboard.putNumber("Shooter/Right/FeedforwardOutput", shooterRightFeedforwardOutput);
    // SmartDashboard.putNumber("Shooter/Right/RightSpeedSetpoint", shooterRightSpeed);
    // SmartDashboard.putBoolean("Shooter/Right/RightReady", readyShoot_Shooter_Right());

    SmartDashboard.putBoolean("Shooter/Elevation/ArriveSetpoint", elevationArriveSetpoint());
    SmartDashboard.putNumber("Shooter/Elevation/AngleSetpoint", elevationSetpoint);
    // SmartDashboard.putNumber("Shooter/Elevation/Temp", getElevationTemp());
    // SmartDashboard.putNumber("Shooter/Elevation/Position/Absolute", getElevationPosition_Absolute());
    // SmartDashboard.putNumber("Shooter/Elevation/Position/Relative", getElevationPosition_Realtive());
    // SmartDashboard.putNumber("Shooter/Elevation/Position/Motor", getElevationPosition_Motor());

    // SmartDashboard.putBoolean("Shooter/Indexer/ready", readyShoot_Indexer());
    // SmartDashboard.putNumber("Shooter/Indexer/targetSpeed", indexerSpeed);
  }
}
