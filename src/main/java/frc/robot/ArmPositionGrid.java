package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(42,7410),//40 + offset
     TOP_MIDDLE(38.0,6355),//36.0 + offset
     TOP_RIGHT(42,7410),//40.0 + offset
     MIDDLE_LEFT(35.0,3570),//34.2 + offset
     MIDDLE_MIDDLE(32.0,2865),//30.0 + offset
     MIDDLE_RIGHT(35.0,3570),//34.2 + offset
     DOWN_LEFT(14.5,3300),//14.5 + offset
     DOWN_MIDDLE(14.5,3300),//14.5 + offset
     DOWN_RIGHT(14.5,3300), //14.5 + offset
     GROUND_PICKUP(16.0, 4600), //16.0 + offset
     SUBSTATION_PICKUP(32, 2000.0), //34.5 + offset
     SUBSTATION_POST_PICKUP(34.0, 2000.0), //34.5 + offset
     STOW(3.0,50); //3 + offset

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
