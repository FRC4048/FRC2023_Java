package frc.robot.utils.logging;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * This command can be used (e.g. assigned to a button) to mark a place in the
 * log. When the drive team finds an issue during competition, pressing the
 * button will place a special string in the log and will allow the pit crew to
 * look for the place in the logs to see what went wrong
 */
public class MarkPlaceCommand extends CommandBase {

	public MarkPlaceCommand() {
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		Logging.instance().traceMessage(Logging.MessageLevel.INFORMATION, "~~~DriveTeam Breakpoint~~~");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {

	}
}
