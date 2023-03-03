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
              SmartShuffleboard.put("DEBUG","CanOpenGripper",safeToOpenGripper());
              SmartShuffleboard.put("DEBUG","estAngle", ProtectionMechanism.armEncoderToAngle(arm.getEncoderValue()));
              SmartShuffleboard.put("DEBUG","extMax", ProtectionMechanism.maxExtenderFromArmAngle(arm.getEncoderValue()));
         }
     }

     private boolean safeToExtend(){
          return extender.getExtenderSensorPos() < maxExtenderFromArmAngle(armEncoderToAngle(arm.getEncoderValue())) && arm.getEncoderValue() > 6;
     }
     public boolean safeToLowerArm(){
          if (arm.getEncoderValue() > 25) return true;
          if (arm.getEncoderValue() < Constants.GRIP_NEEDS_CLOSE_ZONE && gripper.getopenLimitSwitch()) return false;
          return (extender.getExtenderSensorPos() < maxExtenderFromArmAngle(armEncoderToAngle(arm.getEncoderValue())));

     }
     public static double armEncoderToAngle(double value){
          return 17.3 + 0.672 * value - 4.53E-03 * Math.pow(value, 2) + 3.28E-03 * Math.pow(value, 3) - 5.83E-05 * Math.pow(value, 4);
          //17.3 + 0.672x + -4.53E-03x^2 + 3.28E-03x^3 + -5.83E-05x^4
     }

     /**
      * @param value arm encoder value
      * @return the max distance in inches of extender
      */
     public static double maxExtenderFromArmAngle(double value){
          int heightOfArm = 47;
          double armAngle = armEncoderToAngle(value) *  Math.PI/180;
          double maxHeightInches = heightOfArm/Math.cos(armAngle);
          int maxExtEncVal = 7342;
          int minHeightOfExtender = 44;
          int maxHeightOfExtender = 74;
          int ExtenderDiff = maxHeightOfExtender - minHeightOfExtender;
          return (int) (maxExtEncVal * ((maxHeightInches-minHeightOfExtender)/ExtenderDiff));
     }
     
     public boolean safeToOpenGripper(){
          return arm.getEncoderValue() > Constants.GRIP_NEEDS_CLOSE_ZONE;
     }
     public double validateArmVolt(double volt){
          volt = clampVolts(volt,-Constants.ARM_MAX_VOLTS,Constants.ARM_MAX_VOLTS);
          if ((volt < 0 && safeToLowerArm()) || volt > 0) return volt;
          return 0;
     }
     public double validateExtenderVolt(double volt){
          if ((volt > 0 && safeToExtend()) || volt < 0) return volt;
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
