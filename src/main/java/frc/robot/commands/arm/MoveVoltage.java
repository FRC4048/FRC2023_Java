package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Arm;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class MoveVoltage extends LoggedCommand {

    private double voltage;
    private Arm arm;
    private double startTime;
    
    public MoveVoltage(Arm arm, double voltage) {
        this.voltage = voltage;
        this.arm = arm; 
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        super.initialize();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        arm.setVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        return startTime < Timer.getFPGATimestamp() - 5;
    }

    
    
}
