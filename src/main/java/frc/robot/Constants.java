// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static double setMaxOutput(double output, double maxOutput){
    return Math.min(maxOutput, Math.max(-maxOutput, output));
  }

  public static final int motorMaxCurrent = 80;
  public static final double motorMaxOutput = 0.8 * 12;

  public static class ModuleConstants {
    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1 / 5.83;
    public static final double turningGearRatio = 1.0 / (288 / 15);

    public static final double driveVelocityConversionFactor = 
    1 / driveGearRatio * wheelDiameterMeters * Math.PI;

    public static final double drivePositionConversionFactor = 
    driveGearRatio * wheelDiameterMeters * Math.PI;

    public static final double driveEncoderRotMeterPerSec = driveGearRatio * Math.PI * wheelDiameterMeters;
    public static final double driveEncoderRotMeter = driveGearRatio * Math.PI * wheelDiameterMeters;
    public static final double turningEncoderRotRadPerSec = turningGearRatio * 2 * Math.PI;
    public static final double driveEncoderRotMeterPerMin = driveEncoderRotMeterPerSec * 60;
    public static final double driveEncoderRotRadPerMin = turningEncoderRotRadPerSec * 60;

    public static final double kLengthModuleDistance = Units.inchesToMeters(21);
    public static final double kWidthModuleDistance = Units.inchesToMeters(23);

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kLengthModuleDistance/2, kWidthModuleDistance/2),
      new Translation2d(kLengthModuleDistance/2, -kWidthModuleDistance/2),
      new Translation2d(-kLengthModuleDistance/2, kWidthModuleDistance/2),
      new Translation2d(-kLengthModuleDistance/2, -kWidthModuleDistance/2)
    );

    public static final double turningPIDController_kP = 0.013;
    public static final double turningPIDController_kI = 0;
    public static final double turningPIDController_kD = 0.0001;
    public static final double turningPIDController_kC = 0.015;

    public static final double drivePIDntroller_kP = 0;
    public static final double drivePIDController_kI = 0;
    public static final double drivePIDController_kD = 0;

    public static final double driveFeedforward_kS = 0.13;
    public static final double driveFeedforward_kV = 2;
  }

  public class SwerveConstants {
    public static final int leftFrontDrive_ID = 1;
    public static final int leftBackDrive_ID = 2;
    public static final int rightFrontDrive_ID = 4;
    public static final int rightBackDrive_ID = 3;

    public static final int leftFrontTurning_ID = 5;
    public static final int leftBackTurning_ID = 6;
    public static final int rightFrontTurning_ID = 8;
    public static final int rightBackTurning_ID = 7;

    public static final int leftFrontAbsolutedEncoder_ID = 41;
    public static final int leftBackAbsolutedEncoder_ID = 42;
    public static final int rightFrontAbsolutedEncoder_ID = 44;
    public static final int rightBackAbsolutedEncoder_ID = 43;

    public static final double leftFrontOffset = -0.3193359375;
    public static final double leftBackOffset = -0.478271484375;
    public static final double rightFrontOffset = -0.111083984375;
    public static final double rightBackOffset = -0.399169921875;

    public static final int gyro_ID = 55;

    public static final double pathingMoving_kP = 5;
    public static final double pathingMoving_kI = 0;
    public static final double pathingMoving_kD = 0;

    public static final double pathingtheta_kP = 0.8;
    public static final double pathingtheta_kI = 0;
    public static final double pathingtheta_kD = 0;

    public static final double maxDriveSpeed_MeterPerSecond = 5.5;
    public static final double maxAngularVelocity_Angle = 600;

    public static final double kLengthModuleDistance = Units.inchesToMeters(21);
    public static final double kWidthModuleDistance = Units.inchesToMeters(23);

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kLengthModuleDistance/2, kWidthModuleDistance/2),
      new Translation2d(kLengthModuleDistance/2, -kWidthModuleDistance/2),
      new Translation2d(-kLengthModuleDistance/2, kWidthModuleDistance/2),
      new Translation2d(-kLengthModuleDistance/2, -kWidthModuleDistance/2)
    );
  }

  public static class ShooterConstants {
    public static final int indexerMotor_ID = 12;
    public static final int shooterLeftMotor_ID = 14;
    public static final int shooterRightMotor_ID = 13;
    public static final int elevation_ID = 15;
    public static final int elevationCANcoder_ID = 46;
    public static final double elevationOffset = 0;

    public static final double indexerTargetSpeed = 3500;
    public static final double indexerSpeedError = 1500;
    public static final double shooterTargetSpeed = 0;
    public static final double shooterSpeedError = 1000;

    public static final double indexer_kS = 0.323;
    public static final double indexer_kV = 0.09601;

    public static final double shooterLeft_kS = 0.07733;
    public static final double shooterLeft_kV = 0.1104;

    public static final double shooterRight_kS = 0.034349;
    public static final double shooterRight_kV = 0.11046;

    public static final double elevation_kP = 0.8;
    public static final double elevation_kI = 0;
    public static final double elevation_kD = 0;
    public static final double elevation_kS = 0;
    public static final double elevation_kG = 0;
    public static final double elevation_kV = 0;

    public static final double elevationAcceptError = 0.4;
    public static final double hoodMaxAngle = 37;
    public static final double hoodMinAngle = 0;
    public static final double flyWheelMaxSpeed = 6500.0;
    public static final double flyWheelMinSpeed = 0.0;
  }

  public static class ShooterListConstants{
    public static final InterpolatingDoubleTreeMap flyWheelSpeedMap_HUB = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap flyWheelSpeedMap_Home = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap hoodAngleMap_HUB = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap hoodAngleMap_Home = new InterpolatingDoubleTreeMap();
    
    static{
      //HUB
      //Distance, Speed
      // flyWheelSpeedMap_HUB.put(0.8746695103103179, 3750.0);
      // flyWheelSpeedMap_HUB.put(1.631502208517773, 3800.0);
      // flyWheelSpeedMap_HUB.put(1.9710232930686815, 3900.0);
      // flyWheelSpeedMap_HUB.put(2.476387362270586, 4000.0);
      // flyWheelSpeedMap_HUB.put(2.4936686397440258, 4100.0);
      // flyWheelSpeedMap_HUB.put(2.763841884016982, 4490.0);
      // flyWheelSpeedMap_HUB.put(3.1597221213000943, 4500.0);
      // flyWheelSpeedMap_HUB.put(3.608110674190937, 4850.0);
      // flyWheelSpeedMap_HUB.put(5.002913551545545, 4950.0);
      flyWheelSpeedMap_HUB.put(0.7989499650432229, 3950.0);
      flyWheelSpeedMap_HUB.put(1.5589797678661794, 4000.0);
      flyWheelSpeedMap_HUB.put(1.8541082441002856, 4000.0);
      flyWheelSpeedMap_HUB.put(2.6576698039207476, 4300.0);
      flyWheelSpeedMap_HUB.put(2.8228399681908702, 4400.0);
      flyWheelSpeedMap_HUB.put(3.1036631577735085, 4500.0);
      flyWheelSpeedMap_HUB.put(3.488035065090103, 4750.0);
      flyWheelSpeedMap_HUB.put(4.143063397273948,4790.0);
      flyWheelSpeedMap_HUB.put(4.723274298828441, 4850.0);
      
      //Distance, Angle
      // hoodAngleMap_HUB.put(0.8746695103103179, 11.0);
      // hoodAngleMap_HUB.put(1.9710232930686815, 15.0);
      // hoodAngleMap_HUB.put(2.476387362270586, 17.0);
      // hoodAngleMap_HUB.put(2.763841884016982, 17.1);
      // hoodAngleMap_HUB.put(2.7890986188717557, 17.4);
      // hoodAngleMap_HUB.put(3.1597221213000943, 18.0);
      // hoodAngleMap_HUB.put(3.3367156248672476, 18.5);
      // hoodAngleMap_HUB.put(3.608110674190937, 20.0);
      // hoodAngleMap_HUB.put(5.002913551545545, 29.0);
      hoodAngleMap_HUB.put(0.7989499650432229, 6.0);
      hoodAngleMap_HUB.put(1.8541082441002856, 15.0);
      hoodAngleMap_HUB.put(2.6576698039207476, 17.0);
      hoodAngleMap_HUB.put(3.1036631577735085, 18.5);
      hoodAngleMap_HUB.put(3.488035065090103, 20.0);
      hoodAngleMap_HUB.put(4.143063397273948,23.0);
      hoodAngleMap_HUB.put(4.723274298828441, 30.0);
      

      //Home
      //Distance, Speed
      flyWheelSpeedMap_Home.put(4.484607096716331, 3750.0);
      flyWheelSpeedMap_Home.put(7.030488583505097, 4750.0);
      flyWheelSpeedMap_Home.put(8.195414687024375, 5750.0);
      //Distance, Angle
      hoodAngleMap_Home.put(4.484607096716331, 27.0);
      hoodAngleMap_Home.put(7.030488583505097, 27.0);
      hoodAngleMap_Home.put(8.195414687024375, 36.0);
    }

    //Speed
    public static double getSpeed_HUB(double distance){
      return Math.min(ShooterConstants.flyWheelMaxSpeed, Math.max(flyWheelSpeedMap_HUB.get(distance), ShooterConstants.flyWheelMinSpeed));
    }

    public static double getSpeed_Home(double distance){
      return Math.min(ShooterConstants.flyWheelMaxSpeed, Math.max(flyWheelSpeedMap_Home.get(distance), ShooterConstants.flyWheelMinSpeed));
    }

    //Hood
    public static double getHoodAngle_HUB(double distance){
      return Math.min(ShooterConstants.hoodMaxAngle, Math.max(hoodAngleMap_HUB.get(distance), ShooterConstants.hoodMinAngle));
    }

    public static double getHoodAngle_Home(double distance){
      return Math.min(ShooterConstants.hoodMaxAngle, Math.max(hoodAngleMap_Home.get(distance), ShooterConstants.hoodMinAngle));

    }
  }

  public static class ConveyorConstants{
    public static final int conveyorMotor_ID = 11;

    public static final double conveyorTargetSpeed = 0;

    public static final double conveyor_kS = 0;
    public static final double conveyor_kV = 0;
  }

  public static class IntakeConstants {
    public static final int gearMotor_ID = 9;
    public static final int wheelMotor_ID = 10;
    
    public static final double gear_kP = 1.8;
    public static final double gear_kI = 0;
    public static final double gear_kD = 0;
    public static final double gear_kS = 0;
    public static final double gear_kG = 0;
    public static final double gear_kV = 0;

    public static final double primitivePosition = 1;
    public static final double intakeFuelPosition = 41;
    public static final double pushFuelPosition = 20;

    public static final double gearAcceptError = 0.3;
    
    public static final double wheel_kS = 0.24967;
    public static final double wheel_kV = 0.093748;

    public static final double wheelTargetSpeed = 0;
  }

  public static class VisionConstants{
     //meter
    public static final double left_CamtoRobot_TranslationX = 0.245;
    public static final double left_CamtoRobot_TranslationY = 0.2825;
    public static final double left_CamtoRobot_TranslationZ = 0.2237;
    public static final double right_CamtoRobot_TranslationX = 0.245;
    public static final double right_CamtoRobot_TranslationY = -0.2825;
    public static final double right_CamtoRobot_TranslationZ = 0.2237;

    //degree
    public static final double left_CamtoRobot_Pitch = 0;
    public static final double left_CamtoRobot_Roll = 30;
    public static final double left_CamtoRobot_Yaw = 45;
    public static final double right_CamtoRobot_Pitch = 0;
    public static final double right_CamtoRobot_Roll = 30;
    public static final double right_CamtoRobot_Yaw = -45;
  }

  public static class FieldConstants {
    public static final double hub_TranslationX_Blue = Units.inchesToMeters(182.11);
    public static final double hub_TranslationY_Blue = Units.inchesToMeters(158.84);
    public static final double hub_TranslationX_Red = Units.inchesToMeters(471.11);
    public static final double hub_TranslationY_Red = Units.inchesToMeters(158.84);
    public static final double home_TranslationX_Blue = Units.inchesToMeters(72.05);
    public static final double home_TranslationX_Red = Units.inchesToMeters(579.17);
    public static final double allianceZone_TranslationX_Blue = Units.inchesToMeters(159.91);
    public static final double allianceZone_TranslationX_Red = Units.inchesToMeters(491.31);
    public static final double degreeError = 0;
  }

  public static class LEDConstants{
    public static final int candle_ID = 56;
    public static final int ledNumber = 38;
    public static boolean outAlliance = false;
    public static boolean hasTarget = false;
    public static boolean isEnable = false;
    public static boolean shootHome = false;
    public static boolean shootHUB = false;
    public static boolean intake = false;
    public static boolean ledFlag = false;

  }
}