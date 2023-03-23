package frc.robot.commands.extender;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Extender;
import frc.robot.utils.logging.wrappers.LoggedCommand;

public class ManualMoveExtender extends LoggedCommand {
    
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
        extender.move(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (extender.revLimitReached()) extender.resetEncoder();
    }
}
