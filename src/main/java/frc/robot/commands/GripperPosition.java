package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class GripperPosition extends CommandBase {
    private final GripperSubsystem gripper;
    private double startTime, prevTime;
    private double encoderPos;
    private double currentVelocity = 0;
    private double previousVelocity = 0;
    private double slowCounter = 0;

    public GripperPosition(GripperSubsystem gripper) { 
        this.gripper = gripper;
        addRequirements(gripper);
    }

    @Override
    public void end(boolean interrupted) {
        gripper.stop();
    }

    @Override
    public void initialize() {
        
        startTime = Timer.getFPGATimestamp();  
        prevTime = startTime;
        encoderPos = gripper.gripperPosition();
        gripper.close();

    }

    @Override
    public void execute(){
            currentVelocity = Math.abs((gripper.gripperPosition()-encoderPos)/Timer.getFPGATimestamp()-prevTime );
            if (previousVelocity/1.1 > currentVelocity) {
                slowCounter++;
            }
            previousVelocity = currentVelocity;     
            prevTime = Timer.getFPGATimestamp();
            encoderPos = gripper.gripperPosition();
            SmartShuffleboard.put("Gripper", "V", currentVelocity);
        
        
    }

    @Override
    public boolean isFinished() {   
        return slowCounter > Constants.GRIPPER_SLOW_COUNTER || Timer.getFPGATimestamp()-startTime>Constants.GRIPPER_TIMEOUT;
    }   
}
