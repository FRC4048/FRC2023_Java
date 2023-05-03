package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Odometry;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class AlignToSubstation extends LoggedCommand {
     private final Drivetrain drivetrain;
     private final Odometry odometry;
     private int degreeTurnDirection = -1;
     double startTime;
     private boolean arrived = false;

     public AlignToSubstation(Drivetrain drivetrain, Odometry odometry) {
          this.drivetrain = drivetrain;
          this.odometry = odometry;
          addRequirements(drivetrain);
     }

     // Called when the command is initially scheduled.
     @Override
     public void initialize() {
          super.initialize();
          startTime = Timer.getFPGATimestamp();
          double currentDeg = odometry.getOdometry().getEstimatedPosition().getRotation().getDegrees();
          degreeTurnDirection = Math.abs(currentDeg) < 0 ? 1 : -1;
          arrived = false;
     }

     // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
          double currentDegrees = odometry.getOdometry().getEstimatedPosition().getRotation().getDegrees();
          if (Math.abs(currentDegrees) < Constants.SUBSTATION_ALIGN_THRESHOLD) {
               arrived = true;
          }
          else {
               double turnAtSpeed = (Constants.AUTO_TURN_SPEED) * Math.signum(currentDegrees) * degreeTurnDirection;
               drivetrain.drive(0,0, turnAtSpeed, true);
          }
     }

     // Returns true when the command should end.
     @Override
     public boolean isFinished() {
          return arrived || ((Timer.getFPGATimestamp() - startTime) > Constants.AUTO_TURN_TIMEOUT);
     }

     @Override
     public void end(boolean interrupted) {
          super.end(interrupted);
          drivetrain.stopMotors();
     }
}
