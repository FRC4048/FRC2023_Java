package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.HashMap;
import java.util.Map;

public class LogEntry {
    private DataLog dataLog;
    private boolean loggingEnabled;
    private String name;
    private Map<String, DataLogEntry> entryMap = new HashMap<>();

    public LogEntry(String name, boolean enabled) {
        this.name = "/REDSHIFT" + name;
        loggingEnabled = enabled;
        if (loggingEnabled) {
            dataLog = DataLogManager.getLog();
        }
    }

    public void logPose2d(Pose2d pose) {
        if (loggingEnabled) {
            DoubleArrayLogEntry poseEntry = (DoubleArrayLogEntry) entryMap.get(name);
            if (poseEntry == null) {
                poseEntry = new DoubleArrayLogEntry(dataLog, name);
                entryMap.put(name, poseEntry);
            }
            if (pose == null) {
                pose = new Pose2d();
            }
            double[] entry = new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()};
            poseEntry.append(entry);
        }
    }

    public void logDouble(Double value) {
        if (loggingEnabled) {
            DoubleLogEntry doubleEntry = (DoubleLogEntry) entryMap.get(name);
            if (doubleEntry == null) {
                doubleEntry = new DoubleLogEntry(dataLog, name);
                entryMap.put(name, doubleEntry);
            }
            doubleEntry.append(value);
        }
    }

    public void logBoolean(Boolean value) {
        if (loggingEnabled) {
            BooleanLogEntry booleanEntry = (BooleanLogEntry) entryMap.get(name);
            if (booleanEntry == null) {
                booleanEntry = new BooleanLogEntry(dataLog, name);
                entryMap.put(name, booleanEntry);
            }
            booleanEntry.append(value);
        }
    }
}
