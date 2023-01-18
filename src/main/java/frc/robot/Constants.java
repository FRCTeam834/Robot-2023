// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CANIDS {
        public static final int ARM = 10;
    }

    public static final class ArmConstants {
        
    }

    public static final class VisionConstants {
        public static final PoseStrategy POSE_ESTIMATION_STRATEGY = PoseStrategy.CLOSEST_TO_LAST_POSE;
        // Configuration for all cameras
        public static final List<Pair<PhotonCamera, Transform3d>> CAMERAS = new ArrayList<>() {{
            add(new Pair<PhotonCamera, Transform3d>(
                new PhotonCamera("Einstein2023"),
                new Transform3d(
                    new Translation3d(0, 0, 0.5),
                    new Rotation3d(0, 0, 0)
                )
            ));
        }};
    }
}
