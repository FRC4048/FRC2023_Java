package frc.robot.commands.Autonomous;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmPositionGrid;
import frc.robot.Constants;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.sequences.Stow;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;

public class OneGamepiece extends SequentialCommandGroup{
    public OneGamepiece (Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper) {
        addCommands(
        new ResetEncoders(arm, extender),
        new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.TOP_MIDDLE.getArmPosition()),
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                new ExtendToPosition(extender, ArmPositionGrid.TOP_MIDDLE.getExtenderPosition()),
                new OpenGripper(gripper)
            ),
            new HoldArmPID(arm, ArmPositionGrid.TOP_MIDDLE.getArmPosition())
        ),
        new Stow(arm, gripper, extender),
        new MoveDistanceSpinTraj(drivetrain, 0.2, 0.2, Math.toRadians(180)),
        new MoveDistanceSpinTraj(drivetrain, 1.5, 0, Math.toRadians(180))

    );
    }

    
    
}
