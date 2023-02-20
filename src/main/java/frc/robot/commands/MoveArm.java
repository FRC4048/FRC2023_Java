package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    private Arm arm;
    private double setpoint;
    private boolean armMoved=false;
    private double startTime;

    public MoveArm(double setpoint, Arm arm) {
        this.setpoint=setpoint;
        this.arm=arm;
        addRequirements(arm);
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
        arm.setAngle(setpoint);
    }

    @Override
    public boolean isFinished() {
        return (arm.getAngle()>setpoint-3 && arm.getAngle()<setpoint+3) || Timer.getFPGATimestamp() - startTime > 3;
        }
    }
