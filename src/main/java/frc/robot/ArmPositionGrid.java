package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(39.0,7410,-0.5588),
     TOP_MIDDLE(36.0,6355,0),
     TOP_RIGHT(39.0,7410,0.5588),
     MIDDLE_LEFT(34.2,3570,-0.5588),
     MIDDLE_MIDDLE(30.0,2865,0),
     MIDDLE_RIGHT(34.2,3570,0.5588),
     DOWN_LEFT(13.8,3300,-0.5588),
     DOWN_MIDDLE(13.8,3300,0),
     DOWN_RIGHT(13.8,3300,0.5588),
     GROUND_PICKUP(13.0, 4600,-0.5588),
     SUBSTATION_PICKUP(29.4, 2000.0,0),
     SUBSTATION_POST_PICKUP(32.4, 2000.0,0.5588);

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
