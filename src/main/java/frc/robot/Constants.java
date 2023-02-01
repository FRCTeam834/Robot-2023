// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
    // Set to true at competitions
    public static final boolean competitionMode = false;

    public static final boolean telemetryMode = true && !competitionMode;
    /**
     * 
     * ALL UNITS ARE METRIC
     * !Except for angles, which are in degrees
     * 
     */
    public static final class CANIDS {
        public static final int FL_STEER = 1;
        public static final int FL_DRIVE = 2;

        public static final int FR_STEER = 3;
        public static final int FR_DRIVE = 4;

        public static final int BL_STEER = 5;
        public static final int BL_DRIVE = 6;

        public static final int BR_STEER = 7;
        public static final int BR_DRIVE = 8;

        public static final int ELEVATOR_MASTER = 9;
        public static final int ELEVATOR_FOLLOWER = 10;

        public static final int ARM = 11;

        public static final int PIGEON = 13;
    }

    public static final class CHANNELIDS {
        public static final int ELEVATOR_HOMING_LS = 1;
        public static final int ARM_HOMING_LS = 2;
    }

    public static final class ArmConstants {
        public static final double kS = 0.0;
        public static final double kG = 0.0; // Hopefully stays 0 :p
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double GEAR_REDUCTION = 1; // Gear reduction between motor and encoder
        
        public static final int CURRENT_LIMIT = 20;
        public static final int HOME_CURRENT_LIMIT = 40;
        public static final double HOME_SPEED = 0.1;
        public static final double HOMED_POSITION = 1;

        // Softlimits
        public static final double MIN_POSITION = 2;
        public static final double MAX_POSITION = 45;

        public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS =
            new TrapezoidProfile.Constraints(30, 60);
    }

    public static final class ElevatorConstants {
        public static final double kS = 0.0;
        public static final double kG = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;
        public static final double DRUM_RADIUS = 0.05;
        public static final double GEAR_REDUCTION = 1; // Gear reduction between motor and encoder
        
        public static final int CURRENT_LIMIT = 20;
        public static final int HOME_CURRENT_LIMIT = 10;
        public static final double HOME_SPEED = 0.1;

        // Softlimits
        public static final double MIN_POSITION = 0.1;
        public static final double MAX_POSITION = 2;

        // For MOI Calculations
        // Treat carriage as point mass (find the carriage CM)
        public static final double CARRIAGE_MASS = 10;
        public static final double HOMED_DIST_TO_PIVOT = 0.1;
        public static final double STAGE_HEIGHT = 0.5;
        public static final double STAGE_MASS = 5;

        public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS =
            new TrapezoidProfile.Constraints(1, 2);
    }

    public static final class DriveTrainConstants {
        public static final double WIDTH = 1;
        public static final double LENGTH = 1;
        public static final int CURRENT_LIMIT = 20;
        public static final String[] MODULE_NAMES = {
            "FL",
            "FR",
            "BL",
            "BR"
        };
        public static final int[] STEERIDS = {
            CANIDS.FL_STEER,
            CANIDS.FR_STEER,
            CANIDS.BL_STEER,
            CANIDS.BR_STEER
        };
        public static final int[] DRIVEIDS = {
            CANIDS.FL_DRIVE,
            CANIDS.FR_DRIVE,
            CANIDS.BL_DRIVE,
            CANIDS.BR_DRIVE
        };
        public static final Translation2d[] MODULE_POSITIONS = {
            new Translation2d(WIDTH / 2, LENGTH / 2),
            new Translation2d(-WIDTH / 2, LENGTH / 2),
            new Translation2d(WIDTH / 2, -LENGTH / 2),
            new Translation2d(-WIDTH / 2, -LENGTH / 2)
        };
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

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 0;
        public static final double CURRENT_THRESHOLD = 91287821.912;
        public static final int NUM_SAMPLES = 5;
        public static final double MOTOR_SPEED = 2020202.2222;
    }
}
