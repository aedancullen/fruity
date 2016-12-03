package io.github.aedancullen.fruity.system;


public class PathSegment {
    public int id;
    public int success;
    public int fail;

    public double[] navigationTarget;
    public double steeringGain;
    public double[] accuracyThreshold;
    public double basePower;
    public double lowestPower;
    public double powerGain;
    public boolean rampUp;
    public boolean rampDown;
}