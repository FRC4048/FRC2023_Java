package frc.robot.commands.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;

public class ManualMoveArm extends CommandBase {
    
    private Arm arm;
    private DoubleSupplier doubleSupplier;

    public ManualMoveArm(Arm arm, DoubleSupplier doubleSupplier) {
        this.arm = arm;
        this.doubleSupplier = doubleSupplier;
        addRequirements(this.arm);
    }

    @Override
    public void execute() {
        double value = doubleSupplier.getAsDouble();
        // Flipping sign, "down" is positive
        this.arm.manualMove(value * 0.1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
