package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.SmartShuffleboard;

public final class ProtectionMechanism extends SubsystemBase {
     private final Arm arm;
     private final Extender extender;
     private final GripperSubsystem gripper;

     public ProtectionMechanism(Arm arm, Extender extender, GripperSubsystem gripper) {
          this.arm = arm;
          this.extender = extender;
          this.gripper = gripper;
     }

     @Override
     public void periodic() {
         if(Constants.DEBUG){
              SmartShuffleboard.put("DEBUG","CanExtend",safeToExtend());
              SmartShuffleboard.put("DEBUG","CanLowerArm",safeToLowerArm());
              SmartShuffleboard.put("DEBUG","CanZeroArm",safeToZeroArm());
              SmartShuffleboard.put("DEBUG","CanOpenGripper",safeToOpenGripper());
         }
     }

     private boolean safeToExtend(){
          return arm.getEncoderValue() > Constants.NO_EXTENSION_ZONE;
     }
     public boolean safeToLowerArm(){
          return !(arm.getEncoderValue() < Constants.GRIP_NEEDS_CLOSE_ZONE && (gripper.getopenLimitSwitch())) && !(safeToExtend() && extender.getExtenderSensorPos() > Constants.NO_ARM_LOWER_ZONE);
     }
     public boolean safeToZeroArm(){
          return !gripper.getopenLimitSwitch() && safeToLowerArm();
     }
     public boolean safeToOpenGripper(){
          return arm.getEncoderValue() > Constants.GRIP_NEEDS_CLOSE_ZONE;
     }
     public double validateArmVolt(double volt){
          volt = clampVolts(volt,-Constants.ARM_MAX_VOLTS,Constants.ARM_MAX_VOLTS);
          if ((volt < 0 && safeToLowerArm()) || volt > 0) return volt {
               if ((extender.getExtenderSensorPos() > Constants.NO_ARM_LOWER_ZONE) && (volt < 0)) {
                    return 1;
               } else {
               return volt;
               }
          }
          return 0;
     }
     public double validateExtenderVolt(double volt){
          if ((volt < 0 && safeToExtend()) || volt > 0) return volt;
          return 0;
     }
     public double validateGripperVolt(double volt){
          if ((volt > 0 && safeToOpenGripper()) || volt < 0) return volt;
          return 0;
     }
     public double clampVolts(double value, double min, double max){
          return Math.min(Math.max(value, min), max);
     }  
     
}
