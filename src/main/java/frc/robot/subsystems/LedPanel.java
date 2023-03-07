package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public class LedPanel extends SubsystemBase {
    private static int ID;

    private DigitalOutput output1;
    private DigitalOutput output2;
    private DigitalOutput output3;

    public LedPanel() {
        ID = 0;

        output1 = new DigitalOutput(Constants.DIGITAL_OUTPUT_1);
        output2 = new DigitalOutput(Constants.DIGITAL_OUTPUT_2);
        output3 = new DigitalOutput(Constants.DIGITAL_OUTPUT_3);
    }

    public int getID() {
        return ID;
    }

    public void setID(int iD) {
        ID = iD;
        setPic(ID);
    }

    private void setPic(int picID) {
        if(picID == 0) {
            output1.set(false);
            output2.set(false);
            output3.set(false);
        }
        else if(picID == 1) {
            output1.set(true);
            output2.set(false);
            output3.set(false);
        }
        else if(picID == 2) {
            output1.set(false);
            output2.set(true);
            output3.set(false);
        }
        else if(picID == 3) {
            output1.set(true);
            output2.set(true);
            output3.set(false);
        }
        else if(picID == 4) {
            output1.set(false);
            output2.set(false);
            output3.set(true);
        }
        else if(picID == 5) {
            output1.set(true);
            output2.set(false);
            output3.set(true);
        }
        else if(picID == 6) {
            output1.set(false);
            output2.set(true);
            output3.set(true);
        }
        else if(picID == 7) {
            output1.set(true);
            output2.set(true);
            output3.set(true);
        }
    }

    public void periodic() {

    }
}
