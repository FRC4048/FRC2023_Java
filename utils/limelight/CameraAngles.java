

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utils.limelight;

/**
 * Add your docs here.
 */
public class CameraAngles {
    private double tx;
    private double ty;
	public CameraAngles(double tx, double ty) {
        this.tx = tx;
        this.ty = ty;
    }
    
    public double getTx(){
        return tx;
    }
    
    public double getTy(){
        return ty;
    }
}
