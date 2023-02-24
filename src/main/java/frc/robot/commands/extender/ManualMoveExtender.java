package frc.robot.commands.extender;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Extender;

public class ManualMoveExtender extends CommandBase {
    
    private double startTime;
    private Extender extender;
    private DoubleSupplier doubleSupplier;

    public ManualMoveExtender(Extender extender, DoubleSupplier doubleSupplier) {
        this.extender = extender;
        this.doubleSupplier = doubleSupplier;
        addRequirements(this.extender);

    }
    

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }


    @Override
    public void execute() {
        double value = doubleSupplier.getAsDouble();
        // Flipping sign, "down" is positive
        this.extender.move(-value);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > Constants.MANUAL_TIMEOUT;
    }

    
}
