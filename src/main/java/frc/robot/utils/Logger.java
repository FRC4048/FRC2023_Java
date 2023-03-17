package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;

public class Logger {
    private static DataLog dataLog = DataLogManager.getLog();
    private static Map<String, DataLogEntry> entryMap = new HashMap<>();

    public static void logDouble(String topicName, Double value, Boolean logThis) {
        if (!logThis) {
            return;
        }
        DoubleLogEntry doubleEntry = (DoubleLogEntry)getEntry(topicName + "_Double" , (name) -> new DoubleLogEntry(dataLog, name));
        doubleEntry.append(value);
    }

    public static void logBoolean(String topicName, Boolean value, Boolean logThis) {
        if (!logThis) {
            return;
        }
        BooleanLogEntry booleanEntry = (BooleanLogEntry)getEntry(topicName + "_Bool" , (name) -> new BooleanLogEntry(dataLog, name));
        booleanEntry.append(value);
    }

    public static void logPose2d(String topicName, Pose2d value, Boolean logThis) {
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

    private static DataLogEntry getEntry(String name, Function<String,? extends DataLogEntry> func) {
        return entryMap.computeIfAbsent(name, func);
    }
}
