package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Extender;
import frc.robot.utils.SmartShuffleboard;

public class Move extends CommandBase{
    private Arm arm;
    private Extender extender;

    private final ProfiledPIDController m_armPIDController =
      new ProfiledPIDController(
        0.1,
        0,
        0,
          new TrapezoidProfile.Constraints(
              5, 5));
    private final SimpleMotorFeedforward m_armFeedforward = new SimpleMotorFeedforward(0.4, 0.0001);
    
    public Move (Arm arm, Extender extender) {
        this.arm = arm;
        this.extender = extender;
        addRequirements(arm);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    @Override
    public void execute() {
        double armFeedforward = m_armFeedforward.calculate(extender.getEncoder());
        double armControler = m_armPIDController.calculate(arm.getEncoderValue(), arm.getArmSetpoint());
        SmartShuffleboard.put("Arm", "Arm PID", armControler);
        SmartShuffleboard.put("Arm", "Arm Feed Foward", armFeedforward);
        SmartShuffleboard.put("Arm", "Ext Encoder", extender.getEncoder());
    }
    @Override
    public void initialize() {
       
    }
    @Override
    public boolean isFinished() {
        return false;
    }

    

    
    



}
