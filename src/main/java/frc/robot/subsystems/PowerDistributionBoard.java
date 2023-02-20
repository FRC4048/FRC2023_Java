package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SmartShuffleboard;

public class PowerDistributionBoard extends SubsystemBase {
    //PowerDistribution PowerDB = new PowerDistribution(1, ModuleType.kRev);
    PowerDistribution PowerDBTest = new PowerDistribution(0, ModuleType.kCTRE);
    public PowerDistributionBoard() {}
    
    @Override
    public void periodic() {
        // double robotVoltage = PowerDB.getVoltage();
        // SmartShuffleboard.put("Power", "Pannels", "Voltage", robotVoltage);

        // double temperatureCelcius = PowerDB.getTemperature();
        // SmartShuffleboard.put("Power", "Pannels", "Temp in C", temperatureCelcius);

        // double totalCurrent = PowerDB.getTotalCurrent();
        // SmartShuffleboard.put("Power", "Pannels", "Total Current", totalCurrent);

        // double current1 = PowerDB.getCurrent(1);
        // SmartShuffleboard.put("Power", "Current Channels", "Channel 1", current1);

        // double current2 = PowerDB.getCurrent(2);
        // SmartShuffleboard.put("Power", "Current Channels", "Channel 2", current2);
        double robotVoltage = PowerDBTest.getVoltage();
        SmartShuffleboard.put("Power", "Pannels", "Voltage", robotVoltage);

        double temperatureCelcius = PowerDBTest.getTemperature();
        SmartShuffleboard.put("Power", "Pannels", "Temp in C", temperatureCelcius);

        double totalCurrent = PowerDBTest.getTotalCurrent();
        SmartShuffleboard.put("Power", "Pannels", "Total Current", totalCurrent);
        
        // double current1 = PowerDBTest.getCurrent(1);
        // SmartShuffleboard.put("Power", "Current Channels", "Channel 1", current1);

        double current2 = PowerDBTest.getCurrent(2);
        SmartShuffleboard.put("Power", "Current Channels", "Channel 2", current2);
    }
}