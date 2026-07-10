// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final TalonFX turningMotor;
  private final TalonFX driveMotor;

  private final TalonFXConfiguration turningConfig;
  private final TalonFXConfiguration driveConfig;

  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration cancoderConfig;

  private final PIDController turningPidController;
  private final SimpleMotorFeedforward driveFeedForward;

  public SwerveModule(int turningMotor_ID, int driveMotor_ID, int absolutedEncoder_ID, double offset) {
    turningMotor = new TalonFX(turningMotor_ID);
    driveMotor = new TalonFX(driveMotor_ID);

    turningConfig = new TalonFXConfiguration();
    driveConfig = new TalonFXConfiguration();

    absolutedEncoder = new CANcoder(absolutedEncoder_ID);
    cancoderConfig = new CANcoderConfiguration();

    turningPidController = new PIDController(ModuleConstants.turningPIDController_kP, ModuleConstants.turningPIDController_kI, ModuleConstants.turningPIDController_kD);
    turningPidController.enableContinuousInput(ModuleConstants.pidRangeMin, ModuleConstants.pidRangeMax);

    driveFeedForward = new SimpleMotorFeedforward(ModuleConstants.driveFeedforward_kS, ModuleConstants.driveFeedforward_kV);

    turningConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint.Unsigned_0To1;
    cancoderConfig.MagnetSensor.MagnetOffset = offset;

    turningMotor.getConfigurator().apply(turningConfig);
    driveMotor.getConfigurator().apply(driveConfig);
    absolutedEncoder.getConfigurator().apply(cancoderConfig);

    turningMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);

    resetEncoder();
  }

  public void resetEncoder() {
    driveMotor.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningAngle()));
  }

  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble()*ModuleConstants.driveEncoderRotMeter;
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble()*ModuleConstants.driveEncoderRotMeter;//*ModuleConstants.driveEncoderRot2Meter
  }

  public double getDriveMotorTemp(){
    return driveMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getTurningPosition() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getTurningMotorPosition(){
    return turningMotor.getPosition().getValueAsDouble();
  }

  public double getTurningAngle() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }

  public double getTurningMotorTemp(){
    return turningMotor.getDeviceTemp().getValueAsDouble();
  }

  public void stopMotor() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  public void setState(SwerveModuleState state) {
    // Turn Motor
    state.optimize(getState().angle);
    double turningMotorOutput = turningPidController.calculate(getState().angle.getDegrees(), state.angle.getDegrees());
    turningMotor.set(turningMotorOutput);
    // Drive motor
    double driveMotorOutput = driveFeedForward.calculate(state.speedMetersPerSecond)/12;
    driveMotor.set(driveMotorOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
