// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Add your docs here.
 */
public class PowerDistPanel extends SubsystemBase{
    private PowerDistribution pdp;

    public PowerDistPanel() {
        pdp = new PowerDistribution(Constants.PDP_CAN_ID, ModuleType.kCTRE);
	}

    public PowerDistribution getPDP() {
        return pdp;
    }
    
    public void periodic() {
	}
}