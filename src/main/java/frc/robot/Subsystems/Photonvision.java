// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.PhotonConstants;

public class Photonvision extends SubsystemBase {
  private final PhotonCamera shooterCamera = new PhotonCamera("cameraFront");
  PhotonPipelineResult targets;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  Transform3d robotToCam = new Transform3d(new Translation3d(0.0, 0.0, Units.inchesToMeters(24)), new Rotation3d(0,18,0));
  PhotonPoseEstimator photonPoseEstimator;
  /** Creates a new Photonvision. */
  public Photonvision() {
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_RIO, shooterCamera, robotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    targets = shooterCamera.getLatestResult();
  }

  public boolean doesTagExist(int tagID) {
    PhotonTrackedTarget target = null;
    for(int i = 0; i<targets.getTargets().size(); i++) {
      var currentTarget = targets.getTargets().get(i);
      if(currentTarget.getFiducialId() == tagID) {
        target = currentTarget;
      }
    }
    try{
      if(target.getFiducialId() == tagID) {
        return true;
      }else{
        return false;
      }
    } catch(Exception booo) {
      return false;
    }
  }

  public double getTagYaw(int tagID) {
    PhotonTrackedTarget target = null;
    for(int i = 0; i<targets.getTargets().size(); i++) {
      var currentTarget = targets.getTargets().get(i);
      if(currentTarget.getFiducialId() == tagID) {
        target = currentTarget;
      }
    }
    double yaw = target.getYaw();
    return yaw;
  }

  public double getTagDistance(int tagID) {
    PhotonTrackedTarget target = null;
    for(int i = 0; i<targets.getTargets().size(); i++) {
      var currentTarget = targets.getTargets().get(i);
      if(currentTarget.getFiducialId() == tagID) {
        target = currentTarget;
      }
    }
    double distance = PhotonUtils.calculateDistanceToTargetMeters(PhotonConstants.cameraHeight, PhotonConstants.targetHeight, PhotonConstants.cameraPitch, Units.degreesToRadians(target.getPitch()));
    return distance;
  }

  public boolean isTagPresent() {
    return targets.hasTargets();
  }

  public boolean isPoseOnField(Pose2d pose) {
    if(pose.getX() > 0.5 && pose.getY() > 0.5 && pose.getX() <16.06 && pose.getY() < 7.7){
      return true;
    }else{
      return false;
    }
  }

  public EstimatedRobotPose getLatestPoseEstimate() {
    return photonPoseEstimator.update(targets).get();
  }

  public double getLatestPoseLatency() {
    return targets.getTimestampSeconds();
  }

  public void setPipeline(int pipeline) {
    shooterCamera.setPipelineIndex(pipeline);;
  }

  public Vector<N3> confidenceCalculator(EstimatedRobotPose estimation) {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();
      var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance) {
        smallestDistance = distance;
      }
    }
    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
        ? 1
        : Math.max(
          1,
          (estimation.targetsUsed.get(0).getPoseAmbiguity()
              + PhotonConstants.POSE_AMBIGUITY_SHIFTER)
              * PhotonConstants.POSE_AMBIGUITY_MULTIPLIER);
    double confidenceMultiplier = Math.max(
        1,
        (Math.max(
            1,
            Math.max(0, smallestDistance - PhotonConstants.NOISY_DISTANCE_METERS)
                * PhotonConstants.DISTANCE_WEIGHT)
            * poseAmbiguityFactor)
            / (1
                + ((estimation.targetsUsed.size() - 1) * PhotonConstants.TAG_PRESENCE_WEIGHT)));
    
    return PhotonConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
    //return null;
  }
}
