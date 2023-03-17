package frc.robot.commands.arm;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.utils.SmartShuffleboard;

public class MoveArmPIDTest extends CommandBase {

    private Arm arm;
    private Double angle;

    public MoveArmPIDTest(Arm arm, Double angle) {
        this.angle = angle;
        this.arm = arm;
        addRequirements(this.arm);
    }

    @Override
    public void initialize() {
        arm.setPID(Constants.ARM_PID_P_IN, Constants.ARM_PID_I_IN, Constants.ARM_PID_D_IN, Constants.ARM_PID_FF_IN);
        
    }

    @Override
    public void execute() {
        arm.setPidding(true);
        arm.setPIDReference(angle);
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

