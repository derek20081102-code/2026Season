// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  private final PhotonCamera leftCam;
  private final PhotonCamera rightCam;

  private final Transform3d left_CamtoRobot;
  private final Transform3d right_CamtoRobot;

  private final PhotonPoseEstimator left_PoseEstimator;
  private final PhotonPoseEstimator right_PoseEstimator;

  private final AprilTagFieldLayout fieldLayout;

  private Pose2d robotPose;
  private boolean lastHasTarget = false;
  private boolean currentHasTarget = false;
  private String nitama = "false";

  public VisionSubsystem() {
    leftCam = new PhotonCamera("limelight-left");
    rightCam = new PhotonCamera("limelight-right");

    left_CamtoRobot = new Transform3d(
      new Translation3d(VisionConstants.left_CamtoRobot_TranslationX, VisionConstants.left_CamtoRobot_TranslationY, VisionConstants.left_CamtoRobot_TranslationZ),
      new Rotation3d(Math.toRadians(VisionConstants.left_CamtoRobot_Roll), Math.toRadians(VisionConstants.left_CamtoRobot_Pitch), Math.toRadians(VisionConstants.left_CamtoRobot_Yaw))
    );
    right_CamtoRobot = new Transform3d(
      new Translation3d(VisionConstants.right_CamtoRobot_TranslationX, VisionConstants.right_CamtoRobot_TranslationY, VisionConstants.right_CamtoRobot_TranslationZ), 
      new Rotation3d(Math.toRadians(VisionConstants.right_CamtoRobot_Roll), Math.toRadians(VisionConstants.right_CamtoRobot_Pitch), Math.toRadians(VisionConstants.right_CamtoRobot_Yaw))
    );

    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
  
    left_PoseEstimator = new PhotonPoseEstimator(
      fieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      left_CamtoRobot
    );
    right_PoseEstimator = new PhotonPoseEstimator(
      fieldLayout, 
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
      right_CamtoRobot
    );

  }

  private void addIfValid(PhotonCamera camera, PhotonPoseEstimator estimator,SwerveDrivePoseEstimator poseEstimator) {
    var result = camera.getLatestResult();
    if (!result.hasTargets()) return;
    if(result.getBestTarget().area < 0.35) return;
    estimator.update(result).ifPresent(visionPose -> { 
        // if (visionPose.targetsUsed.size() < 2) return;
        poseEstimator.addVisionMeasurement(
          visionPose.estimatedPose.toPose2d(),
          result.getTimestampSeconds()
        );
    });
  }


  public void updateVision(SwerveDrivePoseEstimator poseEstimator) {
    addIfValid(leftCam, left_PoseEstimator, poseEstimator);
    addIfValid(rightCam, right_PoseEstimator, poseEstimator);
  }

  private PhotonPipelineResult rightResult(){
    List<PhotonPipelineResult> result = rightCam.getAllUnreadResults();
    if(result.isEmpty()) return new PhotonPipelineResult();
    return result.get(result.size() - 1);
  }

  private PhotonPipelineResult leftResult(){
    List<PhotonPipelineResult> result = leftCam.getAllUnreadResults();
    if(result.isEmpty()) return new PhotonPipelineResult();
    return result.get(result.size() - 1);
  }

  private double getLeftTargetArea(){
    var result = leftCam.getLatestResult();
    if(result.hasTargets()) return result.getBestTarget().getArea();
    return 0;
  }

  private double getRightTargetArea(){
    var result = rightCam.getLatestResult();
    if(result.hasTargets()) return result.getBestTarget().getArea();
    return 0;
  }

  public boolean leftHasTarget(){
    return getLeftTargetArea() >= 0.35;
  }

  public boolean rightHasTarget(){
    return getRightTargetArea() >= 0.35;
  }

  public void setRobotPose(Pose2d swervePose){
    robotPose = swervePose;
  }

  public boolean isBlueAlliance(){
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }

  public Translation2d getHubToRobot(){
    if(isBlueAlliance())  return new Translation2d(FieldConstants.hub_TranslationX_Blue - robotPose.getX(), FieldConstants.hub_TranslationY_Blue - robotPose.getY());
    return new Translation2d(robotPose.getX() - FieldConstants.hub_TranslationX_Red, robotPose.getY() - FieldConstants.hub_TranslationY_Red);
  }

  public double getRobotToHubAngle(){
    if(isBlueAlliance())  return Units.radiansToDegrees(Math.atan((getHubToRobot().getY())/(getHubToRobot().getX()))) + FieldConstants.degreeError;
    return Units.radiansToDegrees(Math.atan((getHubToRobot().getY())/(getHubToRobot().getX()))) + 180 + FieldConstants.degreeError;
  }

  public double getRobotToHubDistance(){
    return Math.pow(Math.pow(getHubToRobot().getX(), 2) + Math.pow(getHubToRobot().getY(), 2), 0.5);
  }

  public double getRobotToHomeDistance(){
    if(isBlueAlliance())  return robotPose.getX() - FieldConstants.home_TranslationX_Blue;
    return FieldConstants.home_TranslationX_Red - robotPose.getX();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentHasTarget = leftHasTarget() || rightHasTarget();
    LEDConstants.hasTarget = currentHasTarget;
    if(currentHasTarget && !lastHasTarget){
      nitama = "hi";
      LEDConstants.ledFlag = true;
    }
    else if(!currentHasTarget && lastHasTarget){
      nitama = "no";
      LEDConstants.ledFlag = true;
    }
    lastHasTarget = currentHasTarget;
    SmartDashboard.putBoolean("Vision/LeftHasTarget", leftHasTarget());
    SmartDashboard.putBoolean("Vision/rightHasTarget", rightHasTarget());
    SmartDashboard.putBoolean("Vision/lastHasTarget", lastHasTarget);
    SmartDashboard.putBoolean("Vision/currentHasTarget", currentHasTarget);
    SmartDashboard.putNumber("Vision/LeftArea", getLeftTargetArea());
    SmartDashboard.putNumber("Vision/RightArea", getRightTargetArea());
    SmartDashboard.putNumber("Vision/HUBDitance", getRobotToHubDistance());
    SmartDashboard.putNumber("Vision/HomeDistance", getRobotToHomeDistance());
    SmartDashboard.putNumber("Vision/Angle", getRobotToHubAngle());
    // SmartDashboard.putString("nitama", nitama);
  }
}
