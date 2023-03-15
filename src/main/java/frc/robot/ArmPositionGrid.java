package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(42,7410, -0.5588),
     TOP_MIDDLE(39,6355, 0),
     TOP_RIGHT(42,7410, 0.5588),
     MIDDLE_LEFT(37,3570, -0.5588),
     MIDDLE_MIDDLE(33,2865, 0),
     MIDDLE_RIGHT(37,3570, 0.5588),
     DOWN_LEFT(24,3300, -0.5588),
     DOWN_MIDDLE(17,3000, 0),
     DOWN_RIGHT(24,3300, 0.5588);

     private final double armPosition;
     private final double extenderPosition;
     private final double distanceFromTagPosition;

     ArmPositionGrid(double armPosition, double extenderPosition, double distanceFromTagPosition) {
          this.armPosition = armPosition;
          this.extenderPosition = extenderPosition;
          this.distanceFromTagPosition = distanceFromTagPosition;
     }

     public double getArmPosition() {
          return armPosition;
     }

     public double getExtenderPosition() {
          return extenderPosition;
     }

     public double getDistanceFromTagPosition() {
          return distanceFromTagPosition;
     }
}
