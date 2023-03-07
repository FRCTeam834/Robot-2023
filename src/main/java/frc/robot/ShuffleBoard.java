// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.LayoutType;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

/** does things and stuff */
public class ShuffleBoard implements Loggable{
    public class ScoringGrid implements Loggable{
        @Config.ToggleButton(defaultValue = false, tabName = "Operator Interface", name = "Joe Bibutton", methodTypes = {String.class})
        @Config.ToggleButton(defaultValue = false, tabName = "Operator Interface", name = "Barak Obutton", methodTypes = {String.class})
        @Config.ToggleButton(defaultValue = false, tabName = "Operator Interface", name = "George W Button", methodTypes = {String.class})

        public String buttonClicked(boolean enabled, String name){
            return name;
        }
    }
    
    
    @Override
    public LayoutType configureLayoutType() {
        return BuiltInLayouts.kGrid;
    }
    

}
