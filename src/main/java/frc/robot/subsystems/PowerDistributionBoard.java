package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public class PowerDistributionBoard extends SubsystemBase {
    PowerDistribution PowerDB = new PowerDistribution(1, ModuleType.kRev);

    public PowerDistributionBoard() {}
    public void periodic() {
        if (Constants.DEBUG) {
            SmartShuffleboard.put("Power", "Pannels", "Voltage", PowerDB.getVoltage());
            SmartShuffleboard.put("Power", "Pannels", "Temp in C", PowerDB.getTemperature());
            SmartShuffleboard.put("Power", "Pannels", "Total Current", PowerDB.getTotalCurrent());
            SmartShuffleboard.put("Power", "Current Channels", "Channel 1", PowerDB.getCurrent(1));
        }
    }
}