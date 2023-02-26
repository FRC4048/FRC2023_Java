package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public final class Mechanism extends SubsystemBase {
     private static Mechanism mechanism;
     private final Arm arm;
     private final Extender extender;
     private final GripperSubsystem gripper;

     private Mechanism(Arm arm, Extender extender, GripperSubsystem gripper) {
          this.arm = arm;
          this.extender = extender;
          this.gripper = gripper;
     }

     public static Mechanism getInstance() {
          return mechanism;
     }

     public static void newInstance(Arm arm, Extender extender, GripperSubsystem gripper) {
          mechanism = new Mechanism(arm,extender,gripper);
     }

     @Override
     public void periodic() {
          SmartShuffleboard.put("DEBUG","CanExtend",safeToExtend());
          SmartShuffleboard.put("DEBUG","CanLowerArm",safeToLowerArm());
          SmartShuffleboard.put("DEBUG","CanZeroArm",safeToZeroArm());
     }

     public boolean safeToExtend(){
          return arm.getEncoderValue() > Constants.NO_EXTENSION_ZONE;
     }
     public boolean safeToLowerArm(){
          return !(safeToOpenGripper() && gripper.getopenLimitSwitch()) && !(safeToExtend() && extender.getExtenderSensorPos() > Constants.NO_ARM_LOWER_ZONE);
     }
     public boolean safeToZeroArm(){
          return !gripper.getopenLimitSwitch() && safeToLowerArm();
     }
     public boolean safeToOpenGripper(){
          return arm.getEncoderValue() < Constants.GRIP_NEEDS_CLOSE_ZONE;
     }
     
     
     
     
}
