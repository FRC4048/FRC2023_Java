package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;

public class SetPose2d extends CommandBase{

    private Drivetrain drivetrain;
    private Double x;
    private Double y;
    private Double theta;

    public Double getX() {
        return x;
    }


    public void setX(Double x) {
        this.x = x;
    }


    public Double getY() {
        return y;
    }


    public void setY(Double y) {
        this.y = y;
    }


    public Double getTheta() {
        return theta;
    }


    public void setTheta(Double theta) {
        this.theta = theta;
    }

    public SetPose2d(
        Drivetrain drivetrain, int i, int j, int k) {
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
    }


    @Override
    public void initialize() {
        drivetrain.setPose2D(new Pose2d(new Translation2d(x,y),new Rotation2d(theta)));
    }
}
