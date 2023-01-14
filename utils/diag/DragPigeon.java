package frc.robot.utils.diag;

import com.ctre.phoenix.sensors.PigeonIMU;

public class DiagPigeon extends DiagDistanceTraveled {

    private PigeonIMU pigeon;

    public DiagPigeon(String name, int requiredTravel, PigeonIMU pigeon) {
        super(name, requiredTravel);
        this.pigeon = pigeon;

        reset();
    }

    @Override
    protected int getCurrentValue() {
        return (int)pigeon.getFusedHeading();
    }
}
