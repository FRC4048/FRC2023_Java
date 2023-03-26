package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Logger;
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
         if(Constants.ARM_DEBUG || Constants.EXTENDER_DEBUG || Constants.GRIPPER_DEBUG) {
              SmartShuffleboard.put("Extender","CanExtend",safeToExtend());
              SmartShuffleboard.put("Arm","CanLowerArm",safeToLowerArm());
              SmartShuffleboard.put("Gripper","CanOpenGripper",safeToOpenGripper());
         }
         Logger.logBoolean("/protection/safeToExtend", safeToExtend(), Constants.ENABLE_LOGGING);
         Logger.logBoolean("/protection/safeToLowerArm", safeToLowerArm(), Constants.ENABLE_LOGGING);
         Logger.logBoolean("/protection/safeToOpenGripper", safeToOpenGripper(), Constants.ENABLE_LOGGING);
     }

     public boolean safeToExtend(){
          return (arm.getEncoderValue() > Constants.ARM_MONITOR_ZONE) || (extender.getExtenderSensorPos() < maxExtenderFromArmAngle(arm.getEncoderValue()) && arm.getEncoderValue() > Constants.ARM_OUT_ROBOT_MIN);
     }
     public boolean safeToLowerArm(){
          if (arm.getEncoderValue() > Constants.ARM_MONITOR_ZONE) return true;
          if (arm.getEncoderValue() < Constants.GRIP_NEEDS_CLOSE_ZONE && gripper.getopenLimitSwitch()) return false;
          return (extender.getExtenderSensorPos() < maxExtenderFromArmAngle(arm.getEncoderValue()));
     }
     public boolean safeToOpenGripper(){
          return arm.getEncoderValue() > Constants.GRIP_NEEDS_CLOSE_ZONE;
     }
     public double armEncoderToAngle(double value){
          return 17.3 + 0.672 * value - 4.53E-03 * Math.pow(value, 2) + 3.28E-03 * Math.pow(value, 3) - 5.83E-05 * Math.pow(value, 4);
          //17.3 + 0.672x + -4.53E-03x^2 + 3.28E-03x^3 + -5.83E-05x^4
     }

     /**
      * does use if arm angle is greater than 90
      * @param value arm encoder value
      * @return the max distance in inches of extender
      */
     public double maxExtenderFromArmAngle(double value){
          double armAngle = armEncoderToAngle(value) *  Math.PI/180;
          double maxHeightInches = Constants.ARM_HEIGHT/Math.cos(armAngle);
          int extenderDiff = Constants.EXTENDER_MAX_LENGTH - Constants.EXTENDER_MIN_LENGTH;
          return (int) (Constants.MAX_EXTENDER_ENCODER_VALUE * ((maxHeightInches-Constants.EXTENDER_MIN_LENGTH)/extenderDiff));
     }
     
}
