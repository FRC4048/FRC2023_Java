package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class LogEntry {
    private DataLog dataLog;
    boolean loggingEnabled;
    String name;

    public LogEntry(String name, boolean enabled) {
        this.name = "/REDSHIFT" + name;
        loggingEnabled = enabled;
        if (loggingEnabled) {
            dataLog = DataLogManager.getLog();
        }
    }

    public void logPose2d(Pose2d pose) {
        DoubleArrayLogEntry poseEntry = new DoubleArrayLogEntry(dataLog, name);
        if (loggingEnabled) {
            if (pose == null) {
                pose = new Pose2d();
            }
            double[] entry = new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()};
            poseEntry.append(entry);
        }
    }

    public void logDouble(Double value) {
        DoubleLogEntry doubleEntry = new DoubleLogEntry(dataLog, name);
        if (loggingEnabled) {
            doubleEntry.append(value);
        }
    }

    public void logBoolean(Boolean value) {
        BooleanLogEntry booleanEntry = new BooleanLogEntry(dataLog, name);
        if (loggingEnabled) {
            booleanEntry.append(value);
        }
    }
}
