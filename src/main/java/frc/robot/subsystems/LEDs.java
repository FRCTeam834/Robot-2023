// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LedConstants;
import frc.robot.utility.LEDColors;

public class LEDs extends SubsystemBase {

    // Lights! No camera and no action
    private Spark blinkin;

    /** Creates a new LEDs. */
    public LEDs() {

        // Create the Spark object to control the Blinkin
        blinkin = new Spark(LedConstants.PWM_PORT);

        // Default to the beautiful new blue
        blinkin.set(LEDColors.OCEAN);
    }

    /**
     * Sets the color of the LEDs to the specified value
     *
     * @param colorValue The color to set it to
     */
    public void setColor(double colorValue) {
        blinkin.set(colorValue);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
