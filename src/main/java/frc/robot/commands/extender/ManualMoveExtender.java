package frc.robot.commands.extender;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extender;

public class ManualMoveExtender extends CommandBase {
    
    private Extender extender;
    private DoubleSupplier doubleSupplier;

    public ManualMoveExtender(Extender extender, DoubleSupplier doubleSupplier) {
        this.extender = extender;
        this.doubleSupplier = doubleSupplier;
        addRequirements(extender);
    }

    @Override
    public void execute() {
        double speed = doubleSupplier.getAsDouble();
        if (extender.getArm().safeToExtend()) {
        extender.move(speed);
        } else {
        extender.move(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
