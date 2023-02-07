package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.utils.SmartShuffleboard;

public class MatrixSetState {
    private XboxController controller;
    private Button button;
    private int dPadState;

    private boolean up;
    private boolean down;
    private boolean right;
    private boolean left;
    private boolean upRight;
    private boolean upLeft;
    private boolean downRight;
    private boolean downLeft;
    private boolean still;

    public MatrixSetState(XboxController controller) {
        this.controller = controller;
    }


    public void getState() {
        dPadState = controller.getPOV();
        System.out.println(dPadState);
    }

    public void initialize() {
        up = false;
        down = false;
        right = false;
        left = false;
        upRight = false;
        upLeft = false;
        downRight = false;
        downLeft = false;
        still = false;
    }

    public void execute() {
        System.out.println(dPadState);

        if(dPadState == -1) {
            still = true;
        }
        if(dPadState >= 345 || dPadState < 15) {
            up = true;
        }
        if(dPadState >= 30 || dPadState < 60) {
            upRight = true;
        }
        if(dPadState >= 75 || dPadState < 105) {
            right = true;
        }
        if(dPadState >= 120 || dPadState < 150) {
            downRight = true;
        }
        if(dPadState > 165 || dPadState < 195) {
            down = true;
        }
        if(dPadState > 210 || dPadState < 240) {
            downLeft = true;
        }
        if(dPadState > 255 || dPadState < 285) {
            left = true;
        }
        if(dPadState > 300 || dPadState < 330) {
            upLeft = true;
        }

        SmartShuffleboard.put("test", "up", up);
        SmartShuffleboard.put("test", "down", down);
        SmartShuffleboard.put("test", "right", right);
        SmartShuffleboard.put("test", "left", left);
        SmartShuffleboard.put("test", "upRight", upRight);
        SmartShuffleboard.put("test", "upLeft", upLeft);
        SmartShuffleboard.put("test", "downRight", downRight);
        SmartShuffleboard.put("test", "downLeft", downLeft);
        SmartShuffleboard.put("test", "still", still);
    }
}