/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Set;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Add your docs here.
 */
public class SmartShuffleboardTab {
    private Map<String, SimpleWidget> widgetMap = new HashMap();
    private Set<String> commandSet = new HashSet<>();
    private ShuffleboardTab tab;
     
    SmartShuffleboardTab(String tabName) 
    {
        tab = Shuffleboard.getTab(tabName);
    }
         
    public SimpleWidget getWidget(String fieldName)     // return widget handle
    {
        return widgetMap.get(fieldName);
    }

    public ShuffleboardLayout getLayout(String layoutName)     // return layout handle
    {
        try {
            return tab.getLayout(layoutName);
        }   
        catch (Exception noSuchElementException)
        {
            return null;
        }
    }
         
    public void put(String fieldName, Object value)   //primitive
    {
        SimpleWidget widget = widgetMap.get(fieldName);
        if (widget != null)
        {
            GenericEntry ntEntry= widget.getEntry();
            ntEntry.setValue(value);
        }
        else
        {
            widget = tab.add(fieldName, value);
            widgetMap.put(fieldName, widget);
        }
    }
 
    public void putDoubleArray(String fieldName, double[] value)
    {
        SimpleWidget widget = widgetMap.get(fieldName);
        if (widget != null)
        {
            GenericEntry ntEntry= widget.getEntry();
            ntEntry.setDoubleArray(value);
        }
        else
        {
            widget = tab.add(fieldName, value);
            widgetMap.put(fieldName, widget);
        }
    }

    public void put(String fieldName, String layoutName, Object value)   //primitive
    {
        ShuffleboardLayout layout;
        try {
            layout = tab.getLayout(layoutName);
        } catch (NoSuchElementException ex) {
            layout = tab.getLayout(layoutName, BuiltInLayouts.kList);
            layout.withSize(2,4).withProperties(Map.of("Label position", "LEFT"));
        }

        SimpleWidget widget = widgetMap.get(fieldName);
        if (widget != null)
        {
            GenericEntry ntEntry= widget.getEntry();
            ntEntry.setValue(value);
        }
        else
        {
            widget = layout.add(fieldName, value);
            widgetMap.put(fieldName, widget);
        }
    }

    public boolean getBoolean(String fieldName, boolean defaultValue) {
        SimpleWidget widget = widgetMap.get(fieldName);
        if (widget == null) {
            return defaultValue;
        } else {
            return widget.getEntry().getBoolean(defaultValue);
        }
    }

    public double getDouble(String fieldName, double defaultValue) {
        SimpleWidget widget = widgetMap.get(fieldName);
        if (widget == null) {
            return defaultValue;
        } else {
            return widget.getEntry().getDouble(defaultValue);
        }
    }

    public String getString(String fieldName, String defaultValue) {
        SimpleWidget widget = widgetMap.get(fieldName);
        if (widget == null) {
            return defaultValue;
        } else {
            return widget.getEntry().getString(defaultValue);
        }
    }

    public NetworkTableValue getValue(String fieldName) {
        SimpleWidget widget = widgetMap.get(fieldName);
        if (widget == null) {
            return null;
        } else {
            return widget.getEntry().get();
        }
    }

    public void putCommand(String fieldName, CommandBase cmd)
    {
        if (!commandSet.contains(fieldName))
        {
            //getting the layout, or creating it if it doesn't exist
            ShuffleboardLayout layout;
            try {
                layout = tab.getLayout("Commands");
            } catch (NoSuchElementException ex) {
                layout = tab.getLayout("Commands", BuiltInLayouts.kList);
                layout.withSize(2,4).withProperties(Map.of("Label position", "LEFT"));
            }

            //adding the command to the tab
            layout.add(fieldName, cmd);
            commandSet.add(fieldName);
        }
    }
}
