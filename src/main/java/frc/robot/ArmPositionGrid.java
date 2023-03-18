package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(0.65,0), //7410
     TOP_MIDDLE(0.6,0), //6355
     TOP_RIGHT(0.65,0), //7410
     MIDDLE_LEFT(0.57,0), //3570
     MIDDLE_MIDDLE(0.5,0), //2865
     MIDDLE_RIGHT(0.57,0), //3570
     DOWN_LEFT(0.23,0), //3300
     DOWN_MIDDLE(0.23,0), //3300
     DOWN_RIGHT(0.23,0); //3300

     private final double armPosition;
     private final double extenderPosition;

     ArmPositionGrid(double armPosition, double extenderPosition) {
          this.armPosition = armPosition;
          this.extenderPosition = extenderPosition;
     }

     public double getArmPosition() {
          return armPosition;
     }

     public double getExtenderPosition() {
          return extenderPosition;
     }
}
