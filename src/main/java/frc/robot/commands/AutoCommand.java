package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.ArmMoveSequence;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class AutoCommand extends CommandBase {
     private Arm arm;
     private Extender extender;
     private RobotContainer container;

     public AutoCommand(Arm arm, Extender extender, RobotContainer container) {
          this.arm = arm;
          this.extender = extender;
          this.container = container;
     }

     @Override
     public void initialize() {
          CommandScheduler.getInstance().schedule(new ArmMoveSequence(arm, extender, container));
     }

     @Override
     public boolean isFinished() {
          return true;
     }
}
