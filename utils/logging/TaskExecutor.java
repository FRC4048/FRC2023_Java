package frc.robot.utils.logging;

import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

/**
 * This class provides a scheduling facility that allows tasks to be scheduled to be executed repeatedly.
 */
public class TaskExecutor {

    private static final TaskExecutor instance;

    // Initialize the singleton instance. We use static initializer here rather than synchronized lazy initialization
    // in order to pay the initialization price once and not having to have an expensive synchronized block.
    static {
        instance = new TaskExecutor();
    }

    private final ScheduledExecutorService executor = Executors.newScheduledThreadPool(2);
    private final ArrayList<ScheduledFuture<?>> tasks = new ArrayList<ScheduledFuture<?>>();

    static public TaskExecutor instance() {
        return instance;
    }

    /**
     * Schedule a Thread to run with a fixed delay between runs.
     */
    public void scheduleTask(final Runnable task, final long intervalMS) {
        tasks.add(executor.scheduleWithFixedDelay(task, 0, intervalMS, TimeUnit.MILLISECONDS));
    }

    /**
     * Cancel all scheduled threads.
     */
    public void cancelAllTasks() {
        for (final ScheduledFuture<?> task : tasks) {
            task.cancel(true);
        }
        tasks.removeAll(tasks);
    }

}
