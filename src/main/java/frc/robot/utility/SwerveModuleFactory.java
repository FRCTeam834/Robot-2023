// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public class SwerveModuleFactory {
    public static SwerveModule getFrontLeft () {
        return new SwerveModule(1, 5);
    }

    public static SwerveModule getFrontRight () {
        return new SwerveModule(2, 6);
    }

    public static SwerveModule getBackLeft () {
        return new SwerveModule(3, 7);
    }

    public static SwerveModule getBackRight () {
        return new SwerveModule(4, 8);
    }
}
