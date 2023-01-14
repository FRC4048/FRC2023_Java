/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Add your docs here.
 */
public class ColorSensor {
    public enum ColorValue {RED, YELLOW, GREEN, BLUE, UNKNOWN};

    //Accurate measurments for up to 2.5 inches
    private static final Color BLUE_TARGET = new Color(0.135, 0.461, 0.404);     //not makeColor, is in Color Constructor now - fixed
    private static final Color GREEN_TARGET = new Color(0.197, 0.561, 0.240);
    private static final Color RED_TARGET = new Color(0.504, 0.353, 0.144);
    private static final Color YELLOW_TARGET = new Color(0.361, 0.524, 0.113);

    private I2C.Port sensorPort;
    private ColorSensorV3 colorSensor;
    private ColorMatch m_colorMatcher;

    public ColorSensor(I2C.Port sensorPort){
        this.sensorPort = sensorPort;
        colorSensor = new ColorSensorV3(sensorPort);
        m_colorMatcher = new ColorMatch();

        m_colorMatcher.addColorMatch(BLUE_TARGET);
        m_colorMatcher.addColorMatch(GREEN_TARGET);
        m_colorMatcher.addColorMatch(RED_TARGET);
        m_colorMatcher.addColorMatch(YELLOW_TARGET);
    }
    
    //Checks if the color is Red, Blue, Green, Yellow, or Unknown
    public ColorValue getColor(){
        Color detectedColor = colorSensor.getColor();
        
        ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);

        if (match == null){
            return ColorValue.UNKNOWN;
        } 
        if (match.color == BLUE_TARGET) {
            return ColorValue.BLUE;
          } else if (match.color == RED_TARGET) {
            return ColorValue.RED;
          } else if (match.color == GREEN_TARGET) {
            return ColorValue.GREEN;
          } else if (match.color == YELLOW_TARGET) {
            return ColorValue.YELLOW;
          } else {
              return ColorValue.UNKNOWN;
          }
    }
}
