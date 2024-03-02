// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class PhotonConstants {
    public static final double cameraHeight = Units.inchesToMeters(24.0);
    public static final double cameraPitch = Units.degreesToRadians(18.0);
    public static final double targetHeight = Units.inchesToMeters(56.375);

    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.15; //was 0.2
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;
/**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Vector<N3> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

    public static boolean useLimelight = false;
}
