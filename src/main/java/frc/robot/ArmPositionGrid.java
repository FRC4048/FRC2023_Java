package frc.robot;


public enum ArmPositionGrid {
     TOP_LEFT(39,6860),
     TOP_MIDDLE(35,5655),
     TOP_RIGHT(39,6860),
     MIDDLE_LEFT(34,3170),
     MIDDLE_MIDDLE(29,1565),
     MIDDLE_RIGHT(34,3170),
     DOWN_LEFT(22,0),
     DOWN_MIDDLE(22,0),
     DOWN_RIGHT(22,0);

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
