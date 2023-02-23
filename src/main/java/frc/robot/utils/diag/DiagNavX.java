/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.diag;

import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class DiagNavX extends DiagDistanceTraveled {
    private AHRS navX;

    public DiagNavX(String title, String name, double requiredTravel, AHRS navX) {
        super(title, name, requiredTravel);
        this.navX = navX;

        reset();
    }

    @Override
    protected double getCurrentValue() {
        return navX.getAngle();
    }
}
