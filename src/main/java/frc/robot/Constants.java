// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    /* 
        All units are metric unless stated otherwise 
    */

    public static final boolean tuningMode = true && !competitionMode;
    public static final boolean telemetryMode = true && !competitionMode;

    public static final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("Tuning");

    /** Constants pertaining to manual control */
    public static final class DRIVER {
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;

        public static final double DRIVE_SLEWRATE = 2;
        public static final double STEER_SLEWRATE = 4;

        public static final double DRIVE_DEADZONE = 0.075;
        public static final double STEER_DEADZONE = 0.075;
    }

    /** */
    public static final class CANIDS {
        public static final int FL_STEER = 1;
        public static final int FL_DRIVE = 2;
        public static final int FR_STEER = 3;
        public static final int FR_DRIVE = 4;
        public static final int BL_STEER = 5;
        public static final int BL_DRIVE = 6;
        public static final int BR_STEER = 7;
        public static final int BR_DRIVE = 8;
    }

    public static final class PIDGAINS {
        public static final TuneablePIDGains MODULE_STEER = new TuneablePIDGains("Steer Swerve Module", 0.5);
        public static final TuneablePIDGains MODULE_DRIVE = new TuneablePIDGains("Drive Swerve Module", 0.05);
    }

    public static final class DRIVETRAIN {
        public static final double WHEEL_DIAMETER = 0.127;
        public static final double STEER_GEAR_RATIO = 12.8;
        public static final double DRIVE_GEAR_RATIO = 8.14;
        public static final double WIDTH = 0.5;
        public static final double LENGTH = 0.5;

        public static final double MAX_MODULE_SPEED = 3;

        // Minimum speed before module responds
        public static final double MODULE_MOVE_THRESHOLD = 0.1;
        public static final double MODULE_OPTIMIZE_THRESHOLD = 120;
    }

    public static final class POSEESTIMATOR {
        /* Pose estimator standard deviations [x, y, theta] */
        public static final Matrix<N3, N1> STATE_STDDEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            0.0, 0.0, 0.0
        );
        public static final Matrix<N3, N1> VISION_STDDEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            0.0, 0.0, 0.0
        );
    }
}
