package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.NeoSubsystem;
import frc.robot.utils.SmartShuffleboard;

public class NeoDrive extends CommandBase {
    public static final double RUN_TIME = 5.0;

    private String name;
    private NeoSubsystem subsystem;
    private double startTime;
    private double speed;

    public NeoDrive(String name, NeoSubsystem subsystem) {
        this.name = name;
        this.subsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        speed = Constants.MOTOR_TEST_SPEED;
        System.out.println("Driving " + name + " initialized ");
//        speed = SmartShuffleboard.getValue(Constants.TEST_MOTOR, name).getDouble();
    }

    @Override
    public void execute() {
        subsystem.drive(speed);
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
