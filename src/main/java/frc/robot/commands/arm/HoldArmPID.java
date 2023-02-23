package frc.robot.commands.arm;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class HoldArmPID extends CommandBase {
    
    private Arm arm;
    private Double angle;
    private SparkMaxPIDController pidController;

    public HoldArmPID(Arm arm, Double angle) {
        this.angle = angle;
        this.arm = arm;
        addRequirements(this.arm);
        pidController = arm.getNeoMotor().getPIDController();

        SmartShuffleboard.put("Arm", "P Gain", pidController.getP());
        SmartShuffleboard.put("Arm", "I Gain", pidController.getI());
        SmartShuffleboard.put("Arm", "D Gain", pidController.getD());
        SmartShuffleboard.put("Arm", "FF Gain", pidController.getFF());

    }

    @Override
    public void initialize() {
        pidController.setP(Constants.ARM_PID_P_IN, 0);
        pidController.setI(Constants.ARM_PID_I_IN, 0);
        pidController.setD(Constants.ARM_PID_D_IN, 0);
        pidController.setFF(Constants.ARM_PID_FF_IN, 0);

    }

    @Override
    public void execute() {
        arm.setPidding(true);
        if ((arm.getEncoderValue() <= Constants.NO_EXTENSION_ZONE) && !(arm.getExtender().safeToLowerArm())) {
            arm.zeroPID();
        } else {
        pidController.setReference(angle, ControlType.kPosition, 0);
        }
    }

    @Override
    public void end(boolean Interrupted) {
        arm.setPidding(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
