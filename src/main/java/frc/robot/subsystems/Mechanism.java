package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mechanism extends SubsystemBase {
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

     public boolean safeToExtend(){
          return arm.safeToExtend();
     }
     public boolean safeToLowerArm(){
          return extender.safeToLowerArm();
     }
     
     
     
}
