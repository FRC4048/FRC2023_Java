package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ArmPositionGrid;
import frc.robot.AutonomousChooser;
import frc.robot.Constants;
import frc.robot.commands.arm.HoldArmPID;
import frc.robot.commands.arm.VoltageMoveArm;
import frc.robot.commands.extender.ExtendToPosition;
import frc.robot.commands.gripper.OpenGripper;
import frc.robot.commands.sequences.ResetEncoders;
import frc.robot.commands.sequences.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Extender;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.utils.logging.wrappers.ParRaceCommandGroupWrapper;
import frc.robot.utils.logging.wrappers.SequentialCommandGroupWrapper;

public class OneGamepiece extends SequentialCommandGroup{
    
    public OneGamepiece (Drivetrain drivetrain, Arm arm, Extender extender, GripperSubsystem gripper, double yChange, AutonomousChooser.Location location) {
        setName("OneGamepeice");
        addCommands(
        new SequentialCommandGroupWrapper(new ResetEncoders(arm, extender)),
        new VoltageMoveArm(arm, Constants.ARM_AUTO_VOLTAGE_UP, Constants.ARM_AUTO_VOLTAGE_DOWN, ArmPositionGrid.TOP_MIDDLE.getArmPosition()),
        new ParRaceCommandGroupWrapper(new ParallelRaceGroup(
            new SequentialCommandGroup(
                new ExtendToPosition(extender, ArmPositionGrid.TOP_LEFT.getExtenderPosition()),
                new OpenGripper(gripper)
            ),
            new HoldArmPID(arm, ArmPositionGrid.TOP_LEFT.getArmPosition())
        )),

        new SequentialCommandGroupWrapper(new Stow(arm, gripper, extender)),
        new MoveDistanceSpinTraj(drivetrain, 0.2, yChange, Math.toRadians(180)),
        new MoveDistanceSpinTraj(drivetrain, 4.7, 0, Math.toRadians(180))
        //change it back to 4.7

    );
    }

    
    
}
