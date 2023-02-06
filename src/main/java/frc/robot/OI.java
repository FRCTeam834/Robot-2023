// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.DriverConstants;
import frc.robot.utility.UnitQuad;

public class OI {
    public static final Joystick leftJoystick = new Joystick(DriverConstants.LEFT_JOYSTICK_PORT);
    public static final Joystick rightJoystick = new Joystick(DriverConstants.RIGHT_JOYSTICK_PORT);

    /**
     * @return left joystick raw x input
     */
    public static final double getLeftJoystickX () {
        double raw = leftJoystick.getX();
        if (raw < DriverConstants.LEFT_JOYSTICK_DEADZONE) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return left joystick raw y input
     */
    public static final double getLeftJoystickY () {
        double raw = leftJoystick.getY();
        if (raw < DriverConstants.LEFT_JOYSTICK_DEADZONE) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return right joystick raw x input
     */
    public static final double getRightJoystickX () {
        double raw = rightJoystick.getX();
        if (raw < DriverConstants.RIGHT_JOYSTICK_DEADZONE) raw = 0.0;
        return UnitQuad.calculate(raw);
    }

    /**
     * @return right joystick raw y input
     */
    public static final double getRightJoystickY () {
        double raw = rightJoystick.getY();
        if (raw < DriverConstants.RIGHT_JOYSTICK_DEADZONE) raw = 0.0;
        return UnitQuad.calculate(raw);
    }
}