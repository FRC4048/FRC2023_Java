package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Logger {
    private static DataLog dataLog = DataLogManager.getLog();
    private static Map<String, DataLogEntry> entryMap = new HashMap<>();

    public static void logString(String topicName, String value, boolean logThis) {
        if (!logThis) {
            return;
        }
        StringLogEntry stringEntry = (StringLogEntry)getEntry(topicName + "_String" , (name) -> new StringLogEntry(dataLog, name));
        stringEntry.append(value);
    }

    public static void logDouble(String topicName, double value, boolean logThis) {
        if (!logThis) {
            return;
        }
        DoubleLogEntry doubleEntry = (DoubleLogEntry)getEntry(topicName + "_Double" , (name) -> new DoubleLogEntry(dataLog, name));
        doubleEntry.append(value);
    }

    public static void logInteger(String topicName, int value, boolean logThis) {
        if (!logThis) {
            return;
        }
        IntegerLogEntry integerEntry = (IntegerLogEntry)getEntry(topicName + "_Integer", (name) -> new IntegerLogEntry(dataLog, name));
        integerEntry.append(value);
    }

    public static void logBoolean(String topicName, boolean value, boolean logThis) {
        if (!logThis) {
            return;
        }
        BooleanLogEntry booleanEntry = (BooleanLogEntry)getEntry(topicName + "_Bool" , (name) -> new BooleanLogEntry(dataLog, name));
        booleanEntry.append(value);
    }

    public static void logPose2d(String topicName, Pose2d value, boolean logThis) {
        if (!logThis) {
            return;
        }
        DoubleArrayLogEntry poseEntry = (DoubleArrayLogEntry)getEntry(topicName + "_Pose", (name) -> new DoubleArrayLogEntry(dataLog, name));
        if (value == null) {
            value = new Pose2d();
        }
        double[] entry = new double[]{value.getX(), value.getY(), value.getRotation().getRadians()};
        poseEntry.append(entry);
    }

    public static void logTrajectory(String topicName, Trajectory trajectory, boolean logThis) {
        if (!logThis) {
            return;
        }
        if (trajectory == null) {
            return;
        }
        DoubleArrayLogEntry poseEntry = (DoubleArrayLogEntry)getEntry(topicName + "_Trajectory", (name) -> new DoubleArrayLogEntry(dataLog, name));
        double [] entries = new double [trajectory.getStates().size() * 3];
        for (int i=0; i < trajectory.getStates().size(); i = i + 3) {
            State s = trajectory.getStates().get(i);
            Pose2d pose = s.poseMeters;
            entries[i] = (pose.getX());
            entries[i + 1] = (pose.getY());
            entries[i + 2] = (pose.getRotation().getRadians());
        }
        poseEntry.append(entries);
    }

    public static void logTimeout(String commandName, boolean logThis) {
        logString("/robot/timeouts", commandName + " Timed out.", logThis);
    }

    private static DataLogEntry getEntry(String name, Function<String,? extends DataLogEntry> func) {
        return entryMap.computeIfAbsent(name, func);
    }
}
