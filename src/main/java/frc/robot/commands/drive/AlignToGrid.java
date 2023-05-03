package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Odometry;
import frc.robot.utils.Logger;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class AlignToGrid extends LoggedCommand {

    private final Drivetrain drivetrain;
    private final Odometry odometry;

    private double startTime;

    private int degreeTurnDirection = 1;
    
    public AlignToGrid(Drivetrain drivetrain, Odometry odometry) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.odometry = odometry;
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
        double currentDeg = odometry.getOdometry().getEstimatedPosition().getRotation().getDegrees();
        if (Math.signum(currentDeg) == 1) {
            // 0 to 180 , turn positive
            degreeTurnDirection = 1;
        } else {
            // 0 to -180 turn negative
            degreeTurnDirection = -1;
        }
    }

    @Override
    public void execute() {
        double turnAtSpeed = (Constants.AUTO_TURN_SPEED) * degreeTurnDirection;
        drivetrain.drive(0,0, turnAtSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        drivetrain.stopMotors();
    }

    @Override
    public boolean isFinished() {
        double currentDeg = odometry.getOdometry().getEstimatedPosition().getRotation().getDegrees();
        return ((Math.abs((Math.abs(currentDeg) - 180.0)) < Constants.SUBSTATION_ALIGN_THRESHOLD) || 
        ((Timer.getFPGATimestamp() - startTime) > 1.5));
    }    
}
