// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.PhotonConstants;

public class Photonvision extends SubsystemBase {
  private final PhotonCamera shooterCamera = new PhotonCamera("photonvision");
  List<PhotonTrackedTarget> targets;
  /** Creates a new Photonvision. */
  public Photonvision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targets = shooterCamera.getLatestResult().getTargets();
  }

  public boolean doesTagExist(int tagID) {
    PhotonTrackedTarget target = null;
    for(int i = 0; i<targets.size(); i++) {
      var currentTarget = targets.get(i);
      if(currentTarget.getFiducialId() == tagID) {
        target = currentTarget;
      }
    }
    if(target.getFiducialId() == tagID) {
      return true;
    }else{
      return false;
    }
  }

  public double getTagYaw(int tagID) {
    PhotonTrackedTarget target = null;
    for(int i = 0; i<targets.size(); i++) {
      var currentTarget = targets.get(i);
      if(currentTarget.getFiducialId() == tagID) {
        target = currentTarget;
      }
    }
    double yaw = target.getYaw();
    return yaw;
  }

  public double getTagDistance(int tagID) {
    PhotonTrackedTarget target = null;
    for(int i = 0; i<targets.size(); i++) {
      var currentTarget = targets.get(i);
      if(currentTarget.getFiducialId() == tagID) {
        target = currentTarget;
      }
    }
    double distance = PhotonUtils.calculateDistanceToTargetMeters(PhotonConstants.cameraHeight, PhotonConstants.targetHeight, PhotonConstants.cameraPitch, target.getPitch());
    return distance;
  }
}
