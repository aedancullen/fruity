package io.github.aedancullen.fruity.system;
/**

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class FruityAutonomous {

    public FruityController controller;
    public PathFollower pathFollower;

    private PathSegment currentSegment;

    public FruityAutonomous() {}

    public FruityAutonomous(Telemetry telemetry) {
        this.telemetry = telemetry;
        controller = new FruityController(initialPosition, this.telemetry);
    }

    public void beginPathTravel(double[] initialPosition, String pathName) {
        pathFollower = new PathFollower(pathName, this.telemetry);
    }

    public abstract void onSegmentTransition(PathSegment previous, PathSegment next, boolean wasOkayToContinue);

    public abstract boolean shouldContinue(PathSegment segment,
                                           double[] robotAttitude,
                                           double[] robotAcceleration,
                                           double[] robotVelocity,
                                           double[] robotPosition);

    public double[] systemTick() {

        if (controller.getNavigationStatus() == EANHost.ProcessStatus.STOPPED) {
            PathSegment newSegment = pathFollower.moveOnSuccess();
            onSegmentTransition(currentSegment, newSegment, true);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
            }
            else {
                return new double[2];
            }
        }
        else if (shouldContinue(currentSegment,
                host.getRobotAttitude(),
                host.getRobotAcceleration(),
                host.getRobotVelocity(),
                host.getRobotPosition()) == false)
        {
            PathSegment newSegment = pathFollower.moveOnFailure();
            onSegmentTransition(currentSegment, newSegment, false);
            currentSegment = newSegment;
            if (currentSegment != null) {
                host.setNavigationTarget(currentSegment);
            }
            else {
                return new double[2];
            }
        }
        return host.navigationTickDifferential();
    }

}
}
**/