
package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.utils.SmartShuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class matrixCommands extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ExampleSubsystem subsystem;
    private String commands;

    public matrixCommands(ExampleSubsystem subsystem, String commands) {
        this.subsystem = subsystem;
        this.commands = commands;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
      if(commands == "upRight") {
        SmartShuffleboard.put("test", "soham", commands);
      }
      if(commands == "right") {
        SmartShuffleboard.put("test", "soham", commands);
      }
      if(commands == "downRight") {
        SmartShuffleboard.put("test", "soham", commands);
      }
      if(commands == "down") {
        SmartShuffleboard.put("test", "soham", commands);
      }
      if(commands == "downLeft") {
        SmartShuffleboard.put("test", "soham", commands);
      }
      if(commands == "left") {
        SmartShuffleboard.put("test", "soham", commands);
      }
      if(commands == "upLeft") {
        SmartShuffleboard.put("test", "soham", commands);
      }
      if(commands == "still") {
        SmartShuffleboard.put("test", "soham", commands);
      }
    }
  
    @Override
    public void execute() {
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
        return true;
    }
}
