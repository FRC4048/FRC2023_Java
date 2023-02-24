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
        addRequirements(this.extender);

    }
    

    @Override
    public void initialize() {}


    @Override
    public void execute() {
        double value = doubleSupplier.getAsDouble();
        // Flipping sign, "down" is positive
        this.extender.move(-value);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
