// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    public static final double WIDTH = Units.inchesToMeters(35.5);
    public static final double LENGTH = Units.inchesToMeters(35);

    public static final class DriverConstants {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static final int NUMPAD_PORT = 2;
        public static final int XBOX_PORT = 3;

        public static final double LEFT_JOYSTICK_DEADZONE = 0.075;
        public static final double RIGHT_JOYSTICK_DEADZONE = 0.075;
        public static final double XBOX_JOYSTICK_DEADZONE = 0.1;
    }

    public static final class DriveTrainConstants {
        public static final int[][] CANIDS = {
            // {steerID, driveID}
            {2, 3},
            {4, 5},
            {6, 7},
            {8, 9}
        };

        public static final double WIDTH = Units.inchesToMeters(26.5);
        public static final double LENGTH = Units.inchesToMeters(26.5);

        public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);

        public static final double DRIVE_GEAR_RATIO = 4.71429;
        public static final double STEER_GEAR_RATIO = 1;

        public static final double MAX_TRANSLATION_SPEED = Units.feetToMeters(16);
        public static final double MAX_STEER_SPEED = Units.degreesToRadians(360);
        public static final double MAX_MODULE_SPEED = Units.feetToMeters(16);
        /** Minimum speed needed for module to move, mitigates jittering */
        public static final double MODULE_ACTIVATION_THRESHOLD = 0.01;

        public static final PIDGains DRIVE_PID_GAINS = new TuneablePIDGains("SWERVE_DRIVE", 0.5, 0.0);
        public static final PIDGains STEER_PID_GAINS = new TuneablePIDGains("SWERVE_STEER", 0.5, 0.0);
        public static final SimpleMotorFeedforward DRIVE_FF = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

        public static final PIDGains AUTON_DRIVE_PID_GAINS = new PIDGains(1);
        public static final PIDGains AUTON_STEER_PID_GAINS = new PIDGains(2);

        public static final TrapezoidProfile.Constraints AUTON_DRIVE_CONSTRAINTS = new TrapezoidProfile.Constraints(0.5, 1);
        public static final TrapezoidProfile.Constraints AUTON_STEER_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.degreesToRadians(180), Units.degreesToRadians(360));

        /** Slewrate values for drivetrain (max acceleration) */
        public static final double TRANSLATION_SLEWRATE = Units.feetToMeters(32);
        public static final double STEER_SLEWRATE = Units.degreesToRadians(1080);

        public static final PIDGains KEEP_HEADING_PID_GAINS = new PIDGains(Units.degreesToRadians(5));
        public static final double KEEP_HEADING_OMEGA_THRESHOLD = Units.degreesToRadians(5);
        public static final double KEEP_HEADING_TIME_THRESHOLD = 0.1;
        public static final double KEEP_HEADING_SETPOINT_TOLERANCE = Units.degreesToRadians(1);

        public static final int DRIVE_CURRENT_LIMIT = 40;
        public static final int STEER_CURRENT_LIMIT = 20;
        
        public static final String[] MODULE_NAMES = { "FL", "FR", "BL", "BR" };

        public static final Translation2d[] MODULE_POSITIONS = {
            new Translation2d(-WIDTH / 2, LENGTH / 2),
            new Translation2d(WIDTH / 2, LENGTH / 2),
            new Translation2d(-WIDTH / 2, -LENGTH / 2),
            new Translation2d(WIDTH / 2, -LENGTH / 2)
        };

        /*public static final double[] ENCODER_OFFSETS = {
            Units.degreesToRadians(180 - 180),
            Units.degreesToRadians(-90 - 180),
            Units.degreesToRadians(90 - 180),
            Units.degreesToRadians(0 - 180)
        };*/

        public static final class OnTheFlyConstants {

            public static Map<String, Pose2d> WAYPOINTS;
            public static Map<String, Pose2d[][]> PATHS;
            public static Map<String, Pair<Pose2d, Pose2d[][]>> PRESETS;

            public static final void init () {
                WAYPOINTS = new HashMap<String, Pose2d>() {{
                    put("OuterCommunityTop", new Pose2d(5.5, 4.76, Rotation2d.fromDegrees(180)));
                    put("OuterCommunityBottom", new Pose2d(5.5, 0.75, Rotation2d.fromDegrees(180)));
                    put("InnerCommunityTop", new Pose2d(2.32, 4.76, Rotation2d.fromDegrees(180)));
                    put("InnerCommunityBottom", new Pose2d(2.32, 0.75, Rotation2d.fromDegrees(180)));

                    put("ColumnTwoAlign", new Pose2d(2.32, 1.05, Rotation2d.fromDegrees(180)));
                    put("ColumnTwoL13", new Pose2d(1.88, 1.05, Rotation2d.fromDegrees(180)));
                    
                    put("ColumnOneL2", new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
                }};

                // Flip waypoint values if red alliance
                if (DriverStation.getAlliance() == Alliance.Red) {
                    for (Pose2d waypoint : WAYPOINTS.values()) {
                        waypoint.getTranslation().plus(new Translation2d(8.27, 0));
                        waypoint.getRotation().rotateBy(Rotation2d.fromDegrees(180));
                    }
                }

                PATHS = new HashMap<String, Pose2d[][]>() {{
                    put("IntoGriddy", new Pose2d[][]{
                        {
                            WAYPOINTS.get("OuterCommunityTop"),
                            WAYPOINTS.get("OuterCommunityBottom"),
                        },
                        {
                            WAYPOINTS.get("InnerCommunityTop"),
                            WAYPOINTS.get("InnerCommunityBottom"),
                        }
                    });
                }};

                PRESETS = new HashMap<String, Pair<Pose2d, Pose2d[][]>>() {{
                    put("ColumnTwoAlign", new Pair<Pose2d, Pose2d[][]>(
                        WAYPOINTS.get("ColumnTwoAlign"), // Final destination
                        PATHS.get("IntoGriddy") // Path
                    ));
                }};
            }
        }
    }
    
    public static final class PigeonConstants {
        public static final int CANID = 12;
    }

    public static final class ArmConstants {
        public static final int CANID = 10;

        public static final int CURRENT_LIMIT = 50;

        public static final double GEAR_REDUCTION = 180;
        public static final double MAX_POSITION = Units.degreesToRadians(120);
        public static final double MIN_POSITION = Units.degreesToRadians(-40);
        public static final double STARTING_POSITION = Units.degreesToRadians(0);

        public static final TuneablePIDGains PID_GAINS = new TuneablePIDGains("ARM", 55, 2, 5);

        public static final TrapezoidProfile.Constraints PROFILE_CONSTRAINTS = new TrapezoidProfile.Constraints(
            Units.degreesToRadians(240),
            Units.degreesToRadians(360)
        );

        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A20%2C%22u%22%3A%22lbs%22%7D&comLength=%7B%22s%22%3A28.25%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A40%2C%22u%22%3A%22A%22%7D&efficiency=85&endAngle=%7B%22s%22%3A0%2C%22u%22%3A%22deg%22%7D&iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A70%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=%7B%22s%22%3A-90%2C%22u%22%3A%22deg%22%7D
        public static final ArmFeedforward FEEDFORWARD = new ArmFeedforward(0.2, 2.77, 1.36, 0.28);
        public static final double SETPOINT_TOLERANCE = Units.degreesToRadians(0.5);

        public static final double ARM_ANGLE_TO_CB_ARM = Units.degreesToRadians(138);
        public static final double ARM_HEIGHT = Units.inchesToMeters(33);
        public static final double BASE_LENGTH = Units.inchesToMeters(9);
        public static final double ARM_LENGTH = Units.inchesToMeters(30);
        public static final double INTAKE_LENGTH = Units.inchesToMeters(22);
        public static final double CB_ARM_LENGTH = Units.inchesToMeters(8.5);
        public static final double COUNTERBALANCE_FORCE = Units.lbsToKilograms(16);
        public static final double ARM_MASS = Units.lbsToKilograms(10);
        public static final double INTAKE_MASS = Units.lbsToKilograms(10);
        public static final double INTAKE_ANGLE_TO_HORIZONTAL = Units.degreesToRadians(23);

        public static enum ArmPositionPresets {
            STOW(Units.degreesToRadians(-30)),
            L1(Units.degreesToRadians(2)),
            L2(Units.degreesToRadians(90)),
            L3(Units.degreesToRadians(103.9)),
            DS(Units.degreesToRadians(80));

            public final double position;

            ArmPositionPresets (double position) {
                this.position = position;
            }
        };
    }

    public static final class PoseEstimatorConstants {
        /** Standard deviations for odometry and vision (how much we trust their measurements) */
        public static final Matrix<N3, N1> STATE_STDDEVS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(5)); // [x, y, theta]
        public static final Matrix<N3, N1> VISION_STDDEVS = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10)); // [x, y, theta]
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
        public static final int CANID = 11;
        public static final int CURRENT_LIMIT = 20;
        public static final double GEAR_RATIO = 5;
        public static final int RPM_FILTER_TAPS = 30;
        public static final double CONE_RPM_THRESHOLD = 3;
        public static final double CUBE_RPM_THRESHOLD = 10;
        public static final double FREE_RPM_THRESHOLD = 250;
    }

    public static enum GamePieceType {
        NONE,
        CUBE,
        CONE
    };


}
