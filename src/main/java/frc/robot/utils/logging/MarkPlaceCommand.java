package frc.robot.utils.logging;

import frc.robot.utils.logging.wrappers.LoggedCommand;

/**
 * This command can be used (e.g. assigned to a button) to mark a place in the
 * log. When the drive team finds an issue during competition, pressing the
 * button will place a special string in the log and will allow the pit crew to
 * look for the place in the logs to see what went wrong
 */
public class MarkPlaceCommand extends LoggedCommand {

	public MarkPlaceCommand() {
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		super.initialize();
		Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, "~~~DriveTeam Breakpoint~~~");
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return true;
	}
}
