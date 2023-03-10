package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(42,7410),
     TOP_MIDDLE(39,6355),
     TOP_RIGHT(42,7410),
     MIDDLE_LEFT(37,3570),
     MIDDLE_MIDDLE(33,2865),
     MIDDLE_RIGHT(37,3570),
     DOWN_LEFT(24,3300),
     DOWN_MIDDLE(17,3000),
     DOWN_RIGHT(24,3300);

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
