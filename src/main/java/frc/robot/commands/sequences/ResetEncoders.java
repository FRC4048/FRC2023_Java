package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.ResetArm;
import frc.robot.commands.arm.ResetArmEncoder;
import frc.robot.commands.extender.ResetExtenderEncoder;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class ResetEncoders extends SequentialCommandGroup{
    public ResetEncoders (Arm arm, Extender extender){
        setName("ResetEncoders");
        addCommands(
                new ResetExtenderEncoder(extender),
                new ResetArm(arm),
                new ResetArmEncoder(arm)
        );
    }
}
