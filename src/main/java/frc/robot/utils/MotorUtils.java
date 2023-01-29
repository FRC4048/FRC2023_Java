/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class MotorUtils {
    public static final double DEFAULT_TIMEOUT = 0.15;
    private double timeout;
    private double time;
    private PowerDistribution powerDistPanel; 

    final int PDPChannel;
    final double currentThreshold;

    private boolean everStalled = false;

    public MotorUtils(int PDPPort, double currentThreshold, double timeout, PowerDistribution powerDistPanel ){
        this.timeout = timeout;
        this.PDPChannel = PDPPort;
        this.currentThreshold = currentThreshold;
        this.powerDistPanel = powerDistPanel;
        time = Timer.getFPGATimestamp();
    }

    public boolean isStalled() {
        final double currentValue = powerDistPanel.getCurrent(PDPChannel);
        final double now = Timer.getFPGATimestamp();

        if (currentValue < currentThreshold) {
            time = now;
        } else {
            DriverStation.reportError("Motor stall, PDP Channel=" + PDPChannel, false);
            if (now - time > timeout) {
                everStalled = true;
                DataLogManager.log("Motor stall, PDP channel =" + PDPChannel);
                return true;
            }
        }
        return false;
    }

    public boolean everStalled() {
        return everStalled;
    }

    public void resetStall() {
        everStalled = false;
    }
}
