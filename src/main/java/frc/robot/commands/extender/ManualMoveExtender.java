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
        // Flipping sign, "down" is positive
        if (extender.getArm().getEncoderValue() <= 5) {
            speed = 0;
        }
        extender.move(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
