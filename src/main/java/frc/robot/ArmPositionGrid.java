package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(40,7410),
     TOP_MIDDLE(36.0,6355),
     TOP_RIGHT(39.0,7410),
     MIDDLE_LEFT(34.2,3570),
     MIDDLE_MIDDLE(30.0,2865),
     MIDDLE_RIGHT(34.2,3570),
     DOWN_LEFT(14.5,3300),
     DOWN_MIDDLE(14.5,3300),
     DOWN_RIGHT(14.5,3300),
     GROUND_PICKUP(16.0, 4600),
     SUBSTATION_PICKUP(32.5, 2000.0),
     SUBSTATION_POST_PICKUP(34.5, 2000.0),
     STOW(3.0,50);

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
