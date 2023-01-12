// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

/** 2021 misspell legacy */
public class TuneablePIDGains extends PIDGains {

    private final boolean tuningMode = Constants.tuningMode;
    private final NetworkTable tuningTable = Constants.tuningTable;
    private final String name;

    private final TuneableNumber tuneableP;
    private final TuneableNumber tuneableI;
    private final TuneableNumber tuneableD;
    private final boolean isTuneableP;
    private final boolean isTuneableI;
    private final boolean isTuneableD;

    private TuneableNumber tnBuilder(String type, double defaultValue) {
        return new TuneableNumber(this.tuningTable, this.name + "_" + type, defaultValue, this.tuningMode);
    }

    public TuneablePIDGains(String name, double kP) {
        this(name, kP, 0, 0, true, false, false);
    }

    public TuneablePIDGains(String name, double kP, double kD) {
        this(name, kP, 0, kD, true, false, true);
    }

    public TuneablePIDGains(String name, double kP, double kD, double kI) {
        this(name, kP, kI, kD, true, true, true);
    }

    public TuneablePIDGains(String name, double kP, double kI, double kD, boolean isTuneableP, boolean isTuneableI, boolean isTuneableD) {
        super(kP, kI, kD);

        this.name = name;

        this.isTuneableP = isTuneableP;
        this.isTuneableI = isTuneableI;
        this.isTuneableD = isTuneableD;

        if (isTuneableP) {
            tuneableP = this.tnBuilder("P_GAIN", kP);
        } else {
            tuneableP = null;
        }

        if (isTuneableI) {
            tuneableI = this.tnBuilder("I_GAIN", kI);
        } else {
            tuneableI = null;
        }

        if (isTuneableD) {
            tuneableD = this.tnBuilder("D_GAIN", kD);
        } else {
            tuneableD = null;
        }
    }

    @Override
    public double getP() {
        return !(tuningMode && this.isTuneableP) ? this.kP : tuneableP.get();
    }

    @Override
    public double getI() {
        return !(tuningMode && this.isTuneableI) ? this.kI : tuneableI.get();
    }

    @Override
    public double getD() {
        return !(tuningMode && this.isTuneableD) ? this.kD : tuneableD.get();
    }

    public boolean hasChanged() {
        return (
            tuneableP == null ? false : tuneableP.hasChanged() ||
            tuneableI == null ? false : tuneableI.hasChanged() ||
            tuneableD == null ? false : tuneableD.hasChanged()
        );
    }
}
