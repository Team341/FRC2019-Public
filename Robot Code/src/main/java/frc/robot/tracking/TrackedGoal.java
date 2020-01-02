/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tracking;

import java.awt.geom.Point2D;

import frc.robot.RobotMap;
import frc.robot.utilities.DaisyMath;

/**
 * Add your docs here.
 */
public class TrackedGoal {

    private RobotPose mRobotPose;

    private long timeLastUpdated;
    private long timeCreated;

    private int IDnum;

    //private double distanceToGoal; // 2D Range to the Goal in Feet
    private Point2D.Double estPositionOfGoal; //Estimated Position of the Goal in World Frame

    private double estPerpendicularAngleToGoal; //Perpendicular Angle to Goal in Radians
    private double areaRatio;//target area ratio
    private double angle;

    public TrackedGoal(int ID) {
        mRobotPose = RobotPose.getInstance();

        timeCreated = System.currentTimeMillis();
        timeLastUpdated = timeCreated;

        IDnum = ID;

        estPositionOfGoal = new Point2D.Double(0,0);
        estPerpendicularAngleToGoal = 0.0;
    }

    public TrackedGoal(int ID, double distanceToGoal, double perpendicularAngleToGoal, double angleToGoal,
            double AreaRatio) {
        this(ID);
        //update(distanceToGoal, perpendicularAngleToGoal, angleToGoal);

        Point2D.Double relativeDisplacementToGoal = new Point2D.Double();
        relativeDisplacementToGoal.setLocation(distanceToGoal * Math.cos(angleToGoal),
                                                 distanceToGoal * Math.sin(angleToGoal));

        Point2D.Double displacementToGoal = new Point2D.Double();
        displacementToGoal.setLocation(relativeDisplacementToGoal.getX() * Math.cos(mRobotPose.getGyroYaw()) +
                            relativeDisplacementToGoal.getY() * -1.0 * Math.sin(mRobotPose.getGyroYaw()),
                            relativeDisplacementToGoal.getX() * Math.sin(mRobotPose.getGyroYaw()) +
                            relativeDisplacementToGoal.getY() * Math.cos(mRobotPose.getGyroYaw()));

        Point2D.Double positionOfGoal = new Point2D.Double();
        positionOfGoal.setLocation(displacementToGoal.getX() + mRobotPose.getXinFeet(),
                                     displacementToGoal.getY() * mRobotPose.getYinFeet() );

        estPositionOfGoal.setLocation(positionOfGoal.getX(), positionOfGoal.getY());

        estPerpendicularAngleToGoal = Math.asin(displacementToGoal.getX() / distanceToGoal);

        setAreaRatio(AreaRatio);
        setAngle(angleToGoal);
        //System.out.println("ID: " + IDnum + "  Initial X: " + estPositionOfGoal.getX() + "   Inital Y: " + estPositionOfGoal.getY());
    }

    public void update(double distanceToGoal, double perpendicularAngleToGoal, double angleToGoal, double AreaRatio) {

        Point2D.Double relativeDisplacementToGoal = new Point2D.Double();
        relativeDisplacementToGoal.setLocation(distanceToGoal * Math.cos(angleToGoal),
                                                 distanceToGoal * Math.sin(angleToGoal));

        //Rotating Displacement from Robot Frame to World Frame
        Point2D.Double displacementToGoal = new Point2D.Double();
        displacementToGoal.setLocation(
                relativeDisplacementToGoal.getX() * Math.cos(mRobotPose.getGyroYaw())
                        +
                            relativeDisplacementToGoal.getY() * -1.0 * Math.sin(mRobotPose.getGyroYaw()),
                            relativeDisplacementToGoal.getX() * Math.sin(mRobotPose.getGyroYaw()) +
                            relativeDisplacementToGoal.getY() * Math.cos(mRobotPose.getGyroYaw()));

        Point2D.Double positionOfGoal = new Point2D.Double();
        positionOfGoal.setLocation(displacementToGoal.getX() + mRobotPose.getXinFeet(),
                                displacementToGoal.getY() + mRobotPose.getYinFeet() );

        estPositionOfGoal.setLocation(runSmoothingFilter(RobotMap.Vision.GOAL_POSITION_ALPHA,
                                                        estPositionOfGoal.getX(), positionOfGoal.getX()),
                                        runSmoothingFilter(RobotMap.Vision.GOAL_POSITION_ALPHA,
                                                        estPositionOfGoal.getY(), positionOfGoal.getY()));
        estPerpendicularAngleToGoal = Math.asin(displacementToGoal.getX() / distanceToGoal);

        setAreaRatio(AreaRatio);
        setAngle(angleToGoal);

        timeLastUpdated = System.currentTimeMillis();

    }

    public double getRankScore() {
       /* return 0.5 * DaisyMath.minmax(1.0 - (Math.abs(getAngle()) / RobotMap.Vision.GOAL_ANGLE_THRESHOLD_DEGREES), 0.0, 1.0)
                + 0.25 * DaisyMath.minmax(1.0 - ((System.currentTimeMillis() - getTimeLastUpdated()) / RobotMap.Vision.TARGET_TRACK_DROP_THRESHOLD), 0.0, 1.0)
                + 0.25 * DaisyMath.minmax(1.0 - ((System.currentTimeMillis() - getTimeCreated() ) / RobotMap.Vision.TARGET_TRACK_DROP_THRESHOLD), 0.0, 1.0); */
        return DaisyMath.minmax(1.0 - (Math.abs(getAngle()) / RobotMap.Vision.GOAL_ANGLE_THRESHOLD_DEGREES), 0.0, 1.0);
    }

    public double getScore(double distanceToGoal, double angleToGoal) {
        double distScore = 0.0, angleScore = 0.0;

        distScore = DaisyMath.minmax(1.0 - Math.abs((getRange() - distanceToGoal) / RobotMap.Vision.GOAL_DISTANCE_THRESHOLD), 0.0, 1.0);

        angleScore = DaisyMath.minmax(1.0 - (Math.abs(DaisyMath.boundAngleNeg180to180Degrees(getAngleToTarget() - angleToGoal)) / RobotMap.Vision.GOAL_ANGLE_THRESHOLD_RADIANS), 0.0, 1.0);

        return 0.5 * distScore + 0.5 * angleScore;
    }

    public double getRange() {
        double c = Math.pow(mRobotPose.getXinFeet() - estPositionOfGoal.getX(), 2) +
                    Math.pow(mRobotPose.getYinFeet() - estPositionOfGoal.getY(), 2);
        return Math.sqrt(c);
    }

    public double getAngleToTarget() {
        return DaisyMath.boundAngleNegPiToPiRadians(Math.atan2(estPositionOfGoal.getY() - mRobotPose.getYinFeet(),
        estPositionOfGoal.getX() - mRobotPose.getXinFeet()) - Math.toRadians(mRobotPose.getGyroYaw()));
    }

    public double getPerpendicularAngleToTarget() {
        return estPerpendicularAngleToGoal;
    }

    public double getX() {
        return estPositionOfGoal.getX();
    }
    
    public double getY() {
        return estPositionOfGoal.getY();
    }

    public double getAreaRatio() {
        return areaRatio;
    }

    public void setAreaRatio(double areaRatio) {
        this.areaRatio = areaRatio;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public double getTimeLastUpdated() {
        return timeLastUpdated;
    }

    public double getTimeCreated() {
        return timeCreated;
    }

    public int trackID() {
        return IDnum;
    }

    public boolean isDead() {
        return System.currentTimeMillis() > timeLastUpdated + RobotMap.Vision.TARGET_TRACK_DROP_THRESHOLD;
    }

    public static double runSmoothingFilter(double alpha, double oldValue, double newValue) {
        return alpha * oldValue + (1 - alpha) * newValue; //Alpha defined as trust in history
    }
}
