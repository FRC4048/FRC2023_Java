package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.NeoSubsystem;
import frc.robot.subsystems.TalonSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class TalonDrive extends CommandBase {
    public static final double RUN_TIME = 5.0;

    private String name;
    private TalonSubsystem subsystem;
    private double startTime;
    private double speed;

    public TalonDrive(String name, TalonSubsystem subsystem) {
        this.name = name;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        speed = Constants.MOTOR_TEST_SPEED;
//        speed = SmartShuffleboard.getValue(Constants.TEST_MOTOR, name).getDouble();
    }

    @Override
    public void execute() {
        subsystem.drive(speed);
        double checkSpeed = SmartShuffleboard.getDouble(Constants.TEST_MOTOR, "TestSpeed", 0.25);
        if((checkSpeed != speed)) {
            speed = checkSpeed; 
        }
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime) > RUN_TIME;
    }
}
