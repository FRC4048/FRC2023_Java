/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.diag;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.utils.ColorSensor;

/**
 * Add your docs here.
 */
public class DiagColorSensor implements Diagnosable {

    private ColorSensor colorsensor;
    private String name;
    private String title;
    private GenericEntry networkTableEntry;
    private Map<ColorSensor.ColorValue, Boolean> colorMap;

    public DiagColorSensor(String name, String title, ColorSensor colorsensor) {
        this.name = name;
        this.title = title;
        this.colorsensor = colorsensor;
        colorMap = new HashMap<ColorSensor.ColorValue, Boolean>();
        reset();
    }

    @Override
    public void setShuffleBoardTab(ShuffleboardTab shuffleBoardTab, int width, int height) {
        networkTableEntry = shuffleBoardTab.getLayout(title, BuiltInLayouts.kList).withSize(width, height).add(name, false).getEntry();
    }

    @Override
    public void refresh() {
        ColorSensor.ColorValue colorValue = colorsensor.getColor();
        colorMap.put(colorValue, true);
        boolean allColors = colorMap.get(ColorSensor.ColorValue.RED) && colorMap.get(ColorSensor.ColorValue.BLUE)
                            && colorMap.get(ColorSensor.ColorValue.GREEN) && colorMap.get(ColorSensor.ColorValue.YELLOW)
                            && colorMap.get(ColorSensor.ColorValue.UNKNOWN);
        if (networkTableEntry != null) {
            networkTableEntry.setBoolean(allColors);
        }
    }

    @Override
    public void reset() {
        colorMap.put(ColorSensor.ColorValue.RED, false);
        colorMap.put(ColorSensor.ColorValue.GREEN, false);
        colorMap.put(ColorSensor.ColorValue.BLUE, false);
        colorMap.put(ColorSensor.ColorValue.YELLOW, false);
        colorMap.put(ColorSensor.ColorValue.UNKNOWN, false);
    }
}
