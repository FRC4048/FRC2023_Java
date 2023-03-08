package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(39,6860, -0.5588),
     TOP_MIDDLE(35,5655, 0),
     TOP_RIGHT(39,6860, 0.5588),
     MIDDLE_LEFT(34,3170, -0.5588),
     MIDDLE_MIDDLE(29,1565, 0),
     MIDDLE_RIGHT(34,3170, 0.5588),
     DOWN_LEFT(22,0, -0.5588),
     DOWN_MIDDLE(22,0, 0),
     DOWN_RIGHT(22,0, 0.5588);

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
