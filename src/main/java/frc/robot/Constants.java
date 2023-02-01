// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.PIDGains;
import frc.robot.utility.TuneablePIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final boolean competitionMode = false;

    public static final boolean telemetryMode = true && !competitionMode;
    public static final boolean tuningMode = true && !competitionMode;

    /** All units are metric */

    public static final class DriverConstants {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 0;

        public static final double LEFT_JOYSTICK_DEADZONE = 0.075;
        public static final double RIGHT_JOYSTICK_DEADZONE = 0.075;
    }

    public static final class DriveTrainConstants {
        public static final int[][] CANIDS = {
            // {steerID, driveID}
            {1, 2},
            {3, 4},
            {5, 6},
            {7, 8}
        };

        public static final double WIDTH = Units.inchesToMeters(26.5);
        public static final double LENGTH = Units.inchesToMeters(26.5);

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);

        public static final double DRIVE_GEAR_RATIO = 4.71429;
        public static final double STEER_GEAR_RATIO = 1;

        public static final double MAX_TRANSLATION_SPEED = Units.inchesToMeters(3);
        public static final double MAX_STEER_SPEED = Units.degreesToRadians(360);
        public static final double MAX_MODULE_SPEED = Units.inchesToMeters(4);
        /** Minimum speed needed for module to move, mitigates jittering */
        public static final double MODULE_ACTIVATION_THRESHOLD = 0.05;

        public static final PIDGains DRIVE_PID_GAINS = new TuneablePIDGains("SWERVE_DRIVE", 0.0, 0.0);
        public static final PIDGains STEER_PID_GAINS = new TuneablePIDGains("SWERVE_DRIVE", 0.0, 0.0);
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

        public static final PIDGains AUTON_DRIVE_PID_GAINS = new PIDGains(3);
        public static final PIDGains AUTON_STEER_PID_GAINS = new PIDGains(1);

        public static final TrapezoidProfile.Constraints AUTON_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);

        /** Slewrate values for drivetrain (max acceleration) */
        public static final double TRANSLATION_SLEWRATE = 5;
        public static final double STEER_SLEWRATE = Units.degreesToRadians(360);

        public static final PIDGains KEEP_HEADING_PID_GAINS = new PIDGains(Units.degreesToRadians(5));
        public static final double KEEP_HEADING_OMEGA_THRESHOLD = Units.degreesToRadians(5);
        public static final double KEEP_HEADING_TIME_THRESHOLD = 0.1;
        public static final double KEEP_HEADING_SETPOINT_TOLERANCE = Units.degreesToRadians(1);

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int STEER_CURRENT_LIMIT = 20;
        
        public static final String[] MODULE_NAMES = { "FL", "FR", "BL", "BR" };

        public static final Translation2d[] MODULE_POSITIONS = {
            new Translation2d(WIDTH / 2, LENGTH / 2),
            new Translation2d(-WIDTH / 2, LENGTH / 2),
            new Translation2d(WIDTH / 2, -LENGTH / 2),
            new Translation2d(-WIDTH / 2, -LENGTH / 2)
        };
    }

    public static final class PigeonConstants {
        public static final int CANID = 9;
    }

    public static final class ArmConstants {
        public static final int CANID = 10;

        public static final int CURRENT_LIMIT = 40;

        public static final double GEAR_REDUCTION = 100;
        public static final double MAX_POSITION = Units.degreesToRadians(120);
        public static final double MIN_POSITION = Units.degreesToRadians(-30);

        public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            Units.degreesToRadians(180),
            Units.degreesToRadians(360)
        );
        public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);
        public static final double SETPOINT_TOLERANCE = Units.degreesToRadians(0.5);
    }

    public static final class PoseEstimatorConstants {
        /** Standard deviations for odometry and vision (how much we trust their measurements) */
        public static final Matrix<N3, N1> STATE_STDDEVS = VecBuilder.fill(0.2, 0.2, 0.05); // [x, y, theta]
        public static final Matrix<N3, N1> VISION_STDDEVS = VecBuilder.fill(0.7, 0.7, 10); // [x, y, theta]
    }

    public static final class VisionConstants {
        /** 3D Transformation of camera to robot center */
        public static final Transform3d ROBOT_TO_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(0.0, 0.0, 0.0), // x: front/back, y: left/right, z: up/down
            new Rotation3d(0, 0, 0)
        );
        public static final String PHOTON_CAMERA_NAME = "Camera name goes here";
        public static final PoseStrategy POSE_ESTIMATION_STRATEGY = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;
    }

    public static final class IntakeConstants {
        public static final int CANID = 13;
        public static final int CURRENT_LIMIT = 20;
        public static final int RPM_FILTER_TAPS = 12;
        public static final double CONE_RPM_THRESHOLD = 10;
    }
}
