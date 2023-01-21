// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * All units are in metric
     * Angles are in degrees
     */

    /* CAN bus IDs */
    public static final class CANIDS {
        public static final int ARM = 10;
    }

    public static final class CHANNELIDS {
        public static final int ARM_HOMING_LS = 1;
    }

    public static final class ArmConstants {
        /* Feedforward gains (kS must be obtained through sysid) */
        public static final double kS = 1.0;
        public static final double kG = 0.0;
        public static final double kV = 1.0;
        public static final double kA = 1.0;
        /* Gear reduction between motor and encoder; encoder assumed to be 1:1 to mechanism */
        public static final double GEAR_REDUCTION = 1.0;
        /* Max arm velocity and acceleration */
        public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS =
            new TrapezoidProfile.Constraints(1.0, 1.0);
        /* Arm angle (position) softlimits */
        public static final double MIN_POSITION = 5;
        public static final double MAX_POSITION = 90;
        /* Angle of arm when homed */
        public static final double HOME_POSITION = 4;
        /* Current limit */
        public static final int CURENT_LIMIT = 20;
        /* Current limit when homing */
        public static final int HOME_CURRENT_LIMIT = 10;
        /* Percent speed when homing */
        public static final double HOME_SPEED = -0.01;
    }

    public static final class VisionConstants {
        public static final boolean competitionMode = false;
        /* Strategy for resolving pose ambiguity */
        public static final PoseStrategy POSE_ESTIMATION_STRATEGY = PoseStrategy.CLOSEST_TO_LAST_POSE;
        /* Configuration for all cameras */
        public static final List<Pair<PhotonCamera, Transform3d>> CAMERAS = new ArrayList<>() {{
            add(new Pair<PhotonCamera, Transform3d>(
                new PhotonCamera("Einstein2023"),
                new Transform3d(
                    /* Translation from robot center */
                    new Translation3d(0, 0, 0.5),
                    /* Camera heading */
                    new Rotation3d(0, 0, 0)
                )
            ));
        }};
    }
}
