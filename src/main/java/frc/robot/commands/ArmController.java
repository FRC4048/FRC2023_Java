
    public ArmController(Arm arm, double change) {
        this.change = change;
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        desiredAngle = arm.getAngle() + change;
        
        if(desiredAngle > Constants.ARM_MAX_ANGLE) {
            desiredAngle = Constants.ARM_MAX_ANGLE;
        }
        if ((arm.getExtender().getEncoder() > .1) && (desiredAngle < 100)) {
            desiredAngle = arm.getAngle();
        }
        arm.setAngle(desiredAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}