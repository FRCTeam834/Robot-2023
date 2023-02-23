// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

/** Add your docs here. */
public class BoundingBox {
    double x1;
    double x2;
    double y1;
    double y2;
    

    public BoundingBox(double x1, double y1, double x2, double y2){
        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;

        
    }

    public boolean isInBox(double robotX1, double robotY1,double robotX2, double robotY2){
        

        return true;
    }
}
