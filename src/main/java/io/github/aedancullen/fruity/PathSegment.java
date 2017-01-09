package io.github.aedancullen.fruity;


public class PathSegment {
    public int id;
    public int success;
    public int fail;

    public double[] target;
    public double steeringGain;
    public double[] accuracy;
    public double basePower;
    public double lowestPower;
    public double powerGain;
    public boolean rampUp;
    public boolean rampDown;
}