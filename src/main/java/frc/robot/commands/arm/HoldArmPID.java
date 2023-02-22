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
    private boolean pidding;

    public HoldArmPID(Arm arm, Double angle) {
        this.angle = angle;
        this.arm = arm;
        addRequirements(this.arm);
        pidController = arm.getNeoMotor().getPIDController();
        pidController.setP(Constants.ARM_PID_P_IN, 0);
        pidController.setI(Constants.ARM_PID_I_IN, 0);
        pidController.setD(Constants.ARM_PID_D_IN, 0);
        pidController.setFF(Constants.ARM_PID_FF_IN, 0);

        pidController.setP(Constants.ARM_PID_P_IN, 1);
        pidController.setI(Constants.ARM_PID_P_IN, 1);
        pidController.setD(Constants.ARM_PID_P_IN, 1);
        pidController.setFF(Constants.ARM_PID_P_IN, 1);

        SmartShuffleboard.put("PID", "P Gain", pidController.getP());
        SmartShuffleboard.put("PID", "I Gain", pidController.getI());
        SmartShuffleboard.put("PID", "D Gain", pidController.getD());
        SmartShuffleboard.put("PID", "FF Gain", pidController.getFF());

        pidController.setReference(angle, ControlType.kPosition, 0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean Interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
