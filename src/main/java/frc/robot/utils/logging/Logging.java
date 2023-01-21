package frc.robot.utils.logging;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.TimeZone;

public class Logging {

  /**
   * Logger will log every number of iterations specified here
   */
  private static final int LOGGING_FREQ = 1;

  /**
   * Time period in milliseconds between writing to the log file by the dedicated
   * logging thread.
   * Can be overridden by constructor param
   */
  private static final int LOGGING_PERIOD = 20;

  /**
   * Maximum size of the work queue for sending messages to the dedicated logging
   * thread.
   * Can be overridden by constructor param
   */
  private static final int LOGGING_QUEUE_DEPTH = 512;

  /**
   * Logging is a singleton. This is the instance of the class that is used by all logging clients.
   */
  private static final Logging instance;

  // Initialize the logging singleton instance. We use static initializer here rather than synchronized lazy initialization
  // in order to pay the initialization price once and not having to have an expensive synchronized block.
  static {
    instance = new Logging();
  }

  public static enum MessageLevel {
    INFORMATION, TIMER
  }

  // This is TRUE if we failed to write a record to the log in the previous iteration. The idea is that if the queue is
  // full we will lose records but once it is back up a message will appear there to indicate that we missed some records
  private boolean writeLoggingGap = false;

  private final WorkQueue wq;
  private int counter = 0;

  public final static DecimalFormat df5 = new DecimalFormat(".#####");
  public final static DecimalFormat df4 = new DecimalFormat(".####");
  public final static DecimalFormat df3 = new DecimalFormat(".###");
  private final static char COMMA = ',';
  private final static char QUOTE = '"';

  private final static ArrayList<LoggingContext> loggingContexts = new ArrayList<LoggingContext>();
  private static double startTime = 0;

  /**
   * Initialize logger with default settings.
   */
  public Logging() {
    this(LOGGING_PERIOD, new WorkQueue(LOGGING_QUEUE_DEPTH));
  }

  public Logging(long period, WorkQueue wq) {
    this.wq = wq;
    startTime = Timer.getFPGATimestamp();
    TaskExecutor.instance().scheduleTask(new ConsolePrintTask(), period);
  }

  /**
   * @return the singleton instance of this class
   */
  public static Logging instance() {
    return instance;
  }

  /**
   * Reset the startTime of the logging system. This will allow the time to be set to the time of the competition start
   * rather than the robot boot.
   */
  public void setStartTime() {
    startTime = Timer.getFPGATimestamp();
  }


  abstract static public class LoggingContext {
    private int counter = 0;
    private final String subsystem;
    private final StringBuilder sb = new StringBuilder();
    private boolean writeTitles = false;

    public LoggingContext(final String subsystem) {
      this.subsystem = subsystem;
      loggingContexts.add(this);
    }

    public LoggingContext(final Class<?> subsystem) {
      this(subsystem.getSimpleName());
    }

    abstract protected void addAll();

    private final void writeTitles() {
      writeTitles = true;
      writeData();
      writeTitles = false;
    }

    private final void writeData() {
      counter += 1;
      if ((DriverStation.isEnabled() && (counter % LOGGING_FREQ == 0)) || writeTitles) {
        final double now = Timer.getFPGATimestamp();
        sb.setLength(0);
        sb.append(df3.format(now));
        sb.append(COMMA);
        if (DriverStation.isDisabled())
          sb.append(0);
        else
          sb.append(df3.format(now - startTime));
        sb.append(COMMA);
        sb.append(subsystem);
        sb.append(COMMA);
        addAll();
        Logging.instance().traceMessage(sb);
      }
    }

    protected void add(String title, int value) {
      if (writeTitles) {
        sb.append(QUOTE).append(title).append(QUOTE);
      } else {
        sb.append(Integer.toString(value));
      }
      sb.append(COMMA);
    }

    protected void add(String title, boolean value) {
      if (writeTitles) {
        sb.append(QUOTE).append(title).append(QUOTE);
      } else {
        sb.append(Boolean.toString(value));
      }
      sb.append(COMMA);
    }

    protected void add(String title, double value) {
      if (writeTitles) {
        sb.append(QUOTE).append(title).append(QUOTE);
      } else {
        sb.append(Double.toString(value));
      }
      sb.append(COMMA);
    }

    protected void add(String title, String value) {
      if (writeTitles) {
        sb.append(QUOTE).append(title).append(QUOTE);
      } else {
        sb.append(QUOTE).append(value).append(QUOTE);
      }
      sb.append(COMMA);
    }
  }

  private void traceMessage(final StringBuilder sb) {
    if (writeLoggingGap) {
      if (wq.append("LOGGING GAP!!"))
        writeLoggingGap = false;
    }
    if (!wq.append(sb.toString()))
      writeLoggingGap = true;
  }

  public void traceMessage(MessageLevel ml, String... vals) {
    final double now = Timer.getFPGATimestamp();
    final StringBuilder sb = new StringBuilder();
    sb.append(df3.format(now));
    sb.append(COMMA);
    if (DriverStation.isDisabled())
      sb.append(0);
    else
      sb.append(df3.format(now - startTime));
    sb.append(COMMA);
    sb.append(ml.name());
    sb.append(COMMA);
    if (vals != null) {
      for (final String v : vals) {
        sb.append(QUOTE).append(v).append(QUOTE);
        sb.append(COMMA);
      }
    }
    traceMessage(sb);
  }

  /**
   * Iterate through the known logging contexts and write the data for each of
   * them. Logs one logging context every time it'called. It's called by the
   * period() method and we want to spread the cost of logging over multiple calls
   * so we don't run over the 20ms budget.
   */
  public void writeAllData() {
    for (final LoggingContext lc : loggingContexts) {
      lc.writeData();
    }
  }

  /**
   * Iterate through all known logging contexts and write the title for each of
   * them. The #writeAllData and #writeAllTitles functions must iterate through
   * the contexts in the same order so the titles and data are corresponding.
   */
  public void writeAllTitles() {
    for (final LoggingContext lc : loggingContexts) {
      lc.writeTitles();
    }
  }

  private class ConsolePrintTask implements Runnable {
    private PrintWriter log = null;

    public void print() {
      // Log all events, we want this done also when the robot is disabled
      for (;;) {
        final String message = wq.getNext();
        if (message != null)
          log.println(message);
        else
          break;
      }
      log.flush();
    }

    /**
     * Called periodically in its own thread
     */
    public void run() {
      if (log == null) {
        try {
          // TODO: efficiency: Cache file and only create once?
          File file = new File("/home/lvuser/Logs");
          if (!file.exists()) {
            if (file.mkdir()) {
              System.out.println("Log Directory is created!");
            } else {
              System.out.println("Failed to create Log directory!");
            }
          }
          Date date = new Date();
          SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd_ss-SSS");
          dateFormat.setTimeZone(TimeZone.getTimeZone("EST5EDT"));
          try {
            this.log = new PrintWriter("/media/sda1/" + dateFormat.format(date) + "-Log.csv", "UTF-8");
          } catch (Exception e) {
            this.log = new PrintWriter("/home/lvuser/Logs/" + dateFormat.format(date) + "-Log.txt", "UTF-8");
          }

          log.flush();
        } catch (FileNotFoundException e) {
          e.printStackTrace();
        } catch (UnsupportedEncodingException e) {
          e.printStackTrace();
        }

        catch (Exception e) {

          System.out.println(e);

        }
      }
      print();
    }
  }
}
