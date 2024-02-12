// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photonvision extends SubsystemBase {
  private final PhotonCamera shooterCamera = new PhotonCamera("photonvision");
  /** Creates a new Photonvision. */
  public Photonvision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double tx(int tagID) {
    var targets = shooterCamera.getLatestResult().getTargets();
    PhotonTrackedTarget target = shooterCamera.getLatestResult().getBestTarget();
    for(int i = 0; i<14; i++) {
      var currentTarget = targets.get(i);
      if(currentTarget.getFiducialId() == tagID) {
        target = currentTarget;
      }
    }
    double x = target.getYaw();
    return x;
  }
}
