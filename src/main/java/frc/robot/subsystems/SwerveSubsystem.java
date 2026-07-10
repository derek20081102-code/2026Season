package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveModule leftFront;
  private final SwerveModule leftBack;
  private final SwerveModule rightFront;
  private final SwerveModule rightBack;

  private final Pigeon2 gyro;
  private final Pigeon2Configuration gyroConfig;

  private double autoRotation;
  private final Field2d field;

  private RobotConfig robotConfig;
  
  private SwerveDriveOdometry odometry;

  private final SwerveDrivePoseEstimator poseEstimator;
  private final VisionSubsystem m_VisionSubsystem;

  private final PIDController rotationPID;

  public SwerveSubsystem(VisionSubsystem visionSubsystem) {
    this.m_VisionSubsystem = visionSubsystem;

    leftFront = new SwerveModule(
      SwerveConstants.leftFrontTurning_ID,
      SwerveConstants.leftFrontDrive_ID,
      SwerveConstants.leftFrontAbsolutedEncoder_ID,
      SwerveConstants.leftFrontOffset
    );
    rightFront = new SwerveModule(
      SwerveConstants.rightFrontTurning_ID,
      SwerveConstants.rightFrontDrive_ID,
      SwerveConstants.rightFrontAbsolutedEncoder_ID,
      SwerveConstants.rightFrontOffset
    );
    leftBack = new SwerveModule(
      SwerveConstants.leftBackTurning_ID,
      SwerveConstants.leftBackDrive_ID,
      SwerveConstants.leftBackAbsolutedEncoder_ID,
      SwerveConstants.leftBackOffset
    );
    rightBack = new SwerveModule(
      SwerveConstants.rightBackTurning_ID,
      SwerveConstants.rightBackDrive_ID,
      SwerveConstants.rightBackAbsolutedEncoder_ID,
      SwerveConstants.rightBackOffset
    );

    gyro = new Pigeon2(SwerveConstants.gyro_ID);
    gyroConfig = new Pigeon2Configuration();

    gyroConfig.MountPose.MountPosePitch = 0;
    gyroConfig.MountPose.MountPoseRoll = 0;
    gyroConfig.MountPose.MountPoseYaw = 0;

    gyro.getConfigurator().apply(gyroConfig);

      poseEstimator = new SwerveDrivePoseEstimator(
        SwerveConstants.swerveKinematics,
        getRotation(),
        getModulesPosition(),
        new Pose2d()
      );

    poseEstimator.setVisionMeasurementStdDevs(
      VecBuilder.fill(0.5, 0.5, Math.toRadians(10))
    );

    m_VisionSubsystem.setRobotPose(new Pose2d());

    field = new Field2d();

    resetGyro();
                                                       
    try{
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
    // Configure AutoBuilder last
    AutoBuilder.configure(
      this::getRobotPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> autoDrive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(SwerveConstants.pathingMoving_kP, SwerveConstants.pathingMoving_kI, SwerveConstants.pathingMoving_kD), // Translation PID constants
        new PIDConstants(SwerveConstants.pathingtheta_kP, SwerveConstants.pathingtheta_kI, SwerveConstants.pathingtheta_kD) // Rotation PID constants
      ),
      robotConfig, // The robot configuration
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );

    rotationPID = new PIDController(0.008, 0, 0);
    rotationPID.enableContinuousInput(-180, 180);
    odometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getRotation(), getModulesPosition(), new Pose2d());
  }

  public void resetEstimator(){
    poseEstimator.resetPosition(getRotation(), getModulesPosition(), new Pose2d());
  }

  public ChassisSpeeds getChassisSpeed(){
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public Pose2d getRobotPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation(){
    return gyro.getRotation2d();
  }

  public SwerveModulePosition[] getModulesPosition(){
    return new SwerveModulePosition[]{
      leftFront.getPosition(),
      rightFront.getPosition(),
      leftBack.getPosition(),
      rightBack.getPosition()
    };
  }

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[]{
      leftFront.getState(),
      rightFront.getState(),
      leftBack.getState(),
      rightBack.getState()
    };
  }

  public void setModouleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxDriveSpeed_MeterPerSecond);
    leftFront.setState(desiredStates[0]);
    rightFront.setState(desiredStates[1]);
    leftBack.setState(desiredStates[2]);
    rightBack.setState(desiredStates[3]);
  }

  public void setModouleStates_Auto(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxDriveSpeed_MeterPerSecond);
    leftFront.setState(desiredStates[0]);
    rightFront.setState(desiredStates[1]);
    leftBack.setState(desiredStates[2]);
    rightBack.setState(desiredStates[3]);
  }

  public void resetGyro(){
    gyro.reset();
    poseEstimator.resetPosition(getRotation(), getModulesPosition(), new Pose2d());
    // odometry.resetPosition(getRotation(), getModulesPosition(), new Pose2d(0, 0, new Rotation2d()));
  }

  public void resetOdometry(Pose2d poses){
    autoRotation = poses.getRotation().getDegrees();
    poseEstimator.resetPosition(
      getRotation(),
      getModulesPosition(),
      poses
    );
  }

  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient){
    SwerveModuleState[] state;
    xSpeed = xSpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    ySpeed = ySpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    zSpeed = zSpeed * Math.toRadians(SwerveConstants.maxAngularVelocity_Angle);
    if(fieldOrient) {
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation()));
    }else{
      state = SwerveConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }
    setModouleStates(state);
  } 

  public void autoDrive(ChassisSpeeds speeds){
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.01);
    SwerveModuleState[] states = SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);

    setModouleStates(states);
  }

  public double getEstimatedDegree(){
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees() + 7.6;
  }

  public double getRotationPIDOutput_HUB(){
    return Constants.setMaxOutput(rotationPID.calculate(getEstimatedDegree(), m_VisionSubsystem.getRobotToHubAngle()), 0.4);
  }

  public void aimHubRotation(){
    drive(0, 0, getRotationPIDOutput_HUB(), true);
  }

  public void aimHubRotation_Auto(){
    drive(0, 0, getRotationPIDOutput_HUB(), false);
  }

  public double getRotationPIDOutput_Alliance(){
    if(m_VisionSubsystem.isBlueAlliance())  return Constants.setMaxOutput(rotationPID.calculate(getEstimatedDegree() - FieldConstants.degreeError, 180), 0.4);
    return Constants.setMaxOutput(rotationPID.calculate(getEstimatedDegree() - FieldConstants.degreeError, 0), 0.4);
  }

  public void aimHomeRotation(double xSpeed, double ySpeed){
    drive(xSpeed, ySpeed, getRotationPIDOutput_Alliance(), true);
  }

  public boolean arriveAimHubRotation(){
    return Math.abs(rotationPID.getError()) <= 1;
  } 

  public void stop(){
    leftFront.stopMotor();
    rightFront.stopMotor();
    leftBack.stopMotor();
    rightBack.stopMotor();
  }

  public boolean outOfAllianceZone(){
    if(m_VisionSubsystem.isBlueAlliance()) return poseEstimator.getEstimatedPosition().getX() > FieldConstants.allianceZone_TranslationX_Blue;
    return poseEstimator.getEstimatedPosition().getX() < FieldConstants.allianceZone_TranslationX_Red;
  }

  @Override
  public void periodic(){
    poseEstimator.update(
      getRotation(),
      getModulesPosition()
    );

    if(outOfAllianceZone()){
      LEDConstants.outAlliance = true;
    }else{
      LEDConstants.outAlliance = false;
    }

    m_VisionSubsystem.updateVision(poseEstimator);
    m_VisionSubsystem.setRobotPose(poseEstimator.getEstimatedPosition());
    

    field.setRobotPose(poseEstimator.getEstimatedPosition());
    odometry.update(getRotation(), getModulesPosition());

    // SmartDashboard.putNumber("Swerve/leftFront/AbsolutePosion", leftFront.getTurningPosition());
    // SmartDashboard.putNumber("Swerve/leftBack/AbsolutePosion", leftBack.getTurningPosition());
    // SmartDashboard.putNumber("Swerve/rightFront/AbsolutePosion", rightFront.getTurningPosition());
    // SmartDashboard.putNumber("Swerve/rightBack/AbsolutePosion", rightBack.getTurningPosition());

    // SmartDashboard.putNumber("Swerve/leftFront/TurningMotorPosition", leftFront.getTurningMotorPosition());
    // SmartDashboard.putNumber("Swerve/leftBack/TurningMotorPosition", leftBack.getTurningMotorPosition());
    // SmartDashboard.putNumber("Swerve/rightFront/TurningMotorPosition", rightFront.getTurningMotorPosition());
    // SmartDashboard.putNumber("Swerve/rightBack/TurningMotorPosition", rightBack.getTurningMotorPosition());
    
    // SmartDashboard.putNumber("Swerve/leftFront/DrivingMotorPosition", leftFront.getDrivePosition());
    // SmartDashboard.putNumber("Swerve/leftBack/DrivingMotorPosition", leftBack.getDrivePosition());
    // SmartDashboard.putNumber("Swerve/rightFront/DrivingMotorPosition", rightFront.getDrivePosition());
    // SmartDashboard.putNumber("Swerve/rightBack/DrivingMotorPosition", rightBack.getDrivePosition());

    SmartDashboard.putNumber("Swerve/autoRotation", autoRotation);

    SmartDashboard.putNumber("Swerve/gyro", gyro.getYaw().getValueAsDouble());

    // SmartDashboard.putNumber("Swerve/leftFront/TurningMotor/Temp", leftFront.getTurningMotorTemp());
    // SmartDashboard.putNumber("Swerve/leftBack/TurningMotor/Temp", leftBack.getTurningMotorTemp());
    // SmartDashboard.putNumber("Swerve/rightFront/TurningMotor/Temp", rightFront.getTurningMotorTemp());
    // SmartDashboard.putNumber("Swerve/rightBack/TurningMotor/Temp", rightBack.getTurningMotorTemp());

    // SmartDashboard.putNumber("Swerve/leftFront/DriveMotor/Temp", leftFront.getDriveMotorTemp());
    // SmartDashboard.putNumber("Swerve/leftBack/DriveMotor/Temp", leftBack.getDriveMotorTemp());
    // SmartDashboard.putNumber("Swerve/rightFront/DriveMotor/Temp", rightFront.getDriveMotorTemp());
    // SmartDashboard.putNumber("Swerve/rightBack/DriveMotor/Temp", rightBack.getDriveMotorTemp());
    
    SmartDashboard.putNumber("Swerve/nowPoseX_Odmetry", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Swerve/nowPoseY_Odmetry", odometry.getPoseMeters().getY());
    SmartDashboard.putData("Swerve/field", field);


    // SmartDashboard.putNumber("Vision/RotationPIDOutput", getRotationPIDOutput_HUB());
    // SmartDashboard.putNumber("Vision/GoalRotation", m_VisionSubsystem.getRobotToHubAngle());
    // SmartDashboard.putNumber("Vision/NowRotation", getEstimatedDegree());
    // SmartDashboard.putNumber("Vision/NowPoseX_Vision", poseEstimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("Vision/NowPoseY_Vision", poseEstimator.getEstimatedPosition().getY());

    SmartDashboard.putBoolean("Swerve/ArriveAimHubRotation", arriveAimHubRotation());
  }
}