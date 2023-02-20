package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MatrixCommands extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Drivetrain drivetrain;
    private Arm arm;
    private String commands;

    public MatrixCommands(Drivetrain drivetrain, Arm arm,  String commands) {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.commands = commands;
        addRequirements(drivetrain, arm);
    }

    @Override
    public void initialize() {
      if(commands == "up") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_TOP_ROW);
      }
        if(commands == "upRight") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_TOP_ROW);
      }
      if(commands == "right") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_MID_ROW);
      }
      if(commands == "downRight") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_BOTTOM_ROW);
      }
      if(commands == "down") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_BOTTOM_ROW);
      }
      if(commands == "downLeft") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_BOTTOM_ROW);
      }
      if(commands == "left") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_MID_ROW);
      }
      if(commands == "upLeft") {
        SmartShuffleboard.put("test", "soham", commands);
        new SetArmAngle(arm, Constants.ARM_TOP_ROW);
      }
      if(commands == "still") {
        SmartShuffleboard.put("test", "soham", commands);
      }
    }
  
    @Override
    public void execute() {
        if(commands == "up") {}
        if(commands == "upRight") {}
        if(commands == "right") {}
        if(commands == "downRight") {}
        if(commands == "down") {}
        if(commands == "downLeft") {}
        if(commands == "left") {}
        if(commands == "upLeft") {}
        if(commands == "still") {}
    }
  
    @Override
    public void end(boolean interrupted) {
        if(commands == "up") {}
        if(commands == "upRight") {}
        if(commands == "right") {}
        if(commands == "downRight") {}
        if(commands == "down") {}
        if(commands == "downLeft") {}
        if(commands == "left") {}
        if(commands == "upLeft") {}
        if(commands == "still") {}
    }
  
    @Override
    public boolean isFinished() {
        if(commands == "up") {}
        if(commands == "upRight") {}
        if(commands == "right") {}
        if(commands == "downRight") {}
        if(commands == "down") {}
        if(commands == "downLeft") {}
        if(commands == "left") {}
        if(commands == "upLeft") {}
        if(commands == "still") {}
        return true;
    }
}
