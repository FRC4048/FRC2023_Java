package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LedPanel {
    private static int ID;

    private DigitalOutput output1;
    private DigitalOutput output2;
    private DigitalOutput output3;

    public LedPanel() {
        ID = 0;

        output1 = new DigitalOutput(0);
        output2 = new DigitalOutput(0);
        output3 = new DigitalOutput(0);
    }

    public static int getID() {
        return ID;
    }

    public static void setID(int iD) {
        ID = iD;
    }

    public void setPic(int picID) {
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
}
