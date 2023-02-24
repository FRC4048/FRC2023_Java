package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {
    private double time;
    private double startTime;
    public Wait (double time) {
        this.time = time;
    }
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
    }


    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) >= time;
    }
}
