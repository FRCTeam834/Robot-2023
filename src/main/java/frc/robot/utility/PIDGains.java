// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;

/** Helper class for containing PID gain values */
public class PIDGains {
    protected double kP, kI, kD;

    public PIDGains(double kP) {
        this(kP, 0, 0);
    }

    public PIDGains(double kP, double kD) {
        this(kP, 0, kD);
    }

    public PIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getP() {
        return this.kP;
    }

    public double getI() {
        return this.kI;
    }

    public double getD() {
        return this.kD;
    }

    public void bindToController(SparkMaxPIDController controller) {
        controller.setP(this.getP());
        controller.setI(this.getI());
        controller.setD(this.getD());
    }

    public void bindToController(PIDController controller) {
        controller.setP(this.getP());
        controller.setI(this.getI());
        controller.setD(this.getD());
    }

    public boolean hasChanged() {
        return false;
    }

    /** Not useful, interfacing TuneablePIDGains clone */
    public PIDGains clone (String name) {
        return new PIDGains(
            this.getP(),
            this.getI(),
            this.getD()
        );
    }
}
