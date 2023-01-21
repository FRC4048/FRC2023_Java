// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

import java.util.HashMap;
import java.util.Map;


/**
 * This is a wrapper around the Shuffleboard class to make it easier to add data to shuffleboard.
 * Adding and updating a value is done in the same manner, and there's no need to handle networkTable entries.
 * Data can be displayed in its own widget or items can be grouped into a list widget.
 * Example:
 * 
 *                          tab name   list name    field name       value
 *     SmartShuffleboard.put("Drive", "encoders", "Front Right", steerFR.getSelectedSensorPosition(0));
 *     SmartShuffleboard.put("Drive", "encoders", "Front Left", steerFL.getSelectedSensorPosition(0));
 *     SmartShuffleboard.put("Drive", "encoders", "Rear Right", steerRR.getSelectedSensorPosition(0));
 *     SmartShuffleboard.put("Drive", "encoders", "Rear Left", steerRL.getSelectedSensorPosition(0));
 * 
 *                           tab name   field name    value
 *     SmartShuffleboard.put("Drive", "Gyro", getGyro());
 */
public class SmartShuffleboard {

    private final static Map<String, SmartShuffleboardTab> smartTabMap = new HashMap<String, SmartShuffleboardTab>();

    public static void put(String tabName, String fieldName, Object value)    // value is primitive
    {
        SmartShuffleboardTab smartTab = getOrCreateTab(tabName);
        smartTab.put(fieldName, value);
    }

    public static void put(String tabName, String layoutName, String fieldName, Object value)    // value is primitive
    {
        SmartShuffleboardTab smartTab = getOrCreateTab(tabName);
        smartTab.put(fieldName, layoutName, value);
    }

    public static boolean getBoolean(String tabName, String fieldName, boolean defaultValue) {
        SmartShuffleboardTab smartTab = smartTabMap.get(tabName);
        if (smartTab == null) {
            return defaultValue;
        }
        return smartTab.getBoolean(fieldName, defaultValue);
    }

    public static double getDouble(String tabName, String fieldName, double defaultValue) {
        SmartShuffleboardTab smartTab = smartTabMap.get(tabName);
        if (smartTab == null) {
            return defaultValue;
        }
        return smartTab.getDouble(fieldName, defaultValue);
    }

    public static String getString(String tabName, String fieldName, String defaultValue) {
        SmartShuffleboardTab smartTab = smartTabMap.get(tabName);
        if (smartTab == null) {
            return defaultValue;
        }
        return smartTab.getString(fieldName, defaultValue);
    }

    public static NetworkTableValue getValue(String tabName, String fieldName) {
        SmartShuffleboardTab smartTab = smartTabMap.get(tabName);
        if (smartTab == null) {
            return null;
        }
        return smartTab.getValue(fieldName);
    }

    public static void putCommand(String tabName, String fieldName, CommandBase cmd)    // value is primitive
    {
        SmartShuffleboardTab smartTab = getOrCreateTab(tabName);
        smartTab.putCommand(fieldName, cmd);
    }

    public static SimpleWidget getWidget(String tabName, String fieldName)
    {
        SmartShuffleboardTab tab = smartTabMap.get(tabName);

        if (tab == null)
        {
            return null;
        }
        else
        {
            return tab.getWidget(fieldName);
        }
    }

    public static ShuffleboardLayout getLayout(String tabName, String layoutName)
    {
        SmartShuffleboardTab tab = smartTabMap.get(tabName);

        if (tab == null)
        {
            return null;
        }
        else
        {
            return tab.getLayout(layoutName);
        }
    }

    private static SmartShuffleboardTab getOrCreateTab(String tabName) {
        SmartShuffleboardTab smartTab = smartTabMap.get(tabName);

        if (smartTab == null) {
            smartTab = new SmartShuffleboardTab(tabName);
            smartTabMap.put(tabName, smartTab);
        }
        return smartTab;
    }

}
