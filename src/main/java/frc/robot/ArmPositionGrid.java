package frc.robot;

public enum ArmPositionGrid {
     TOP_LEFT(35.0,7150),
     TOP_MIDDLE(32.0,7000),
     TOP_RIGHT(35.0,7150),
     MIDDLE_LEFT(30,3500),
     MIDDLE_MIDDLE(26.5,2965),
     MIDDLE_RIGHT(30,3500),
     DOWN_LEFT(14.5,3300),
     DOWN_MIDDLE(14.5,3300),
     DOWN_RIGHT(14.5,3300),
     GROUND_PICKUP(16.0, 4600),
     SUBSTATION_PICKUP(28.5, 2000.0),
     SUBSTATION_POST_PICKUP(30.5, 2000.0),
     STOW(0,50);

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
