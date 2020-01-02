/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.tracking;

import java.awt.Point;
import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class GoalTracking {

    private static GoalTracking mInstance;
    private TrackedGoal lastBestTrack;

    public static GoalTracking getInstance() {
      if (mInstance == null)
        mInstance = new GoalTracking();
      return mInstance;
    }

    private RobotPose mRobotPose;

    private double distanceToGoal; // 2D Range to the Goal in Feet
    private Point relativeDisplacementToGoal; //Estimated X Y Vector to the Goal in Feet in Robot Frame
    private Point displacementToGoal; //Estimated X Y Vector to the Goal in Feet in World Frame
    private Point positionOfGoal; //Estimated Position of the Goal in World Frame

    private double perpendicularAngleToGoal; //Perpendicular Angle to Goal in Radians
    private double angleToGoal; //Line of Sight to the Target from the Robot in Radians
    private double areaRatio; //ratio of the area of the target

    private ArrayList<TrackedGoal> trackList;
    private TrackedGoal bestTrack;
    private int trackCounter, count;
    private double numOfTargets;

    private double tv; //Whether the SunLight has any valid targets (0 or 1)
    private double tx; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
    private double ty; //Vertical Offset from Crosshair to Target (-20.5 to 20.5)
    private double ta; //Target Area (0% of image to 100% of image)

    public GoalTracking() {
        //SmartDashboard.putString("DriveToVisionTarget/Check Constructor Start", "We made it!");
        mRobotPose = RobotPose.getInstance();

        trackList = new ArrayList<TrackedGoal>();
        bestTrack = null;
        trackCounter = 0;

        distanceToGoal = -999.999;
        relativeDisplacementToGoal = new Point(0, 0);
        positionOfGoal = new Point(0, 0);
        perpendicularAngleToGoal = 0.0;
        angleToGoal = 0.0;
        count = 0;
        numOfTargets = 0.0;
        lastBestTrack = null;
        tv = 0.0;
        tx = 0.0;
        ty = 0.0;
        ta = 0.0;
        //SmartDashboard.putString("DriveToVisionTarget/Check Constructor Exit", "We made it!");
    }

    public void run() {
        numOfTargets = NetworkTable.getTable("limelight").getDouble("tv", 0);
        SmartDashboard.putNumber("Vision/A Number of Targets", numOfTargets);
        SmartDashboard.putBoolean("Vision/Target Acquired", numOfTargets == 1);
        if(numOfTargets > 0) {
            //SmartDashboard.putString("DriveToVisionTarget/A Start of run If statement", "We made it!");
            for(double i = 0; i < numOfTargets; i++)
            {
                getReport(i);
                if(!matchReportToTrack()) {
                    //create new track with data
                    trackList.add(new TrackedGoal(trackCounter, distanceToGoal, perpendicularAngleToGoal, angleToGoal, areaRatio));
                    trackCounter++;
                }
            }
        } else {
            count++;
        }

        pruneDeadTracks(); //removes outdated tracks

        selectBestTrack();

        logToDashboard();
    }

    public void getReport(double x) {
        //Gets measurements from Pi
        //SmartDashboard.putString("DriveToVisionTarget/A Start of getReport", "We made it!");
        //distanceToGoal = NetworkTable.getTable("GRIP/Target/Vision ID: " + (x + 1.0)).getNumber("Distance", -1.0);
        tv = NetworkTable.getTable("limelight").getDouble("tv", 0);
        tx = NetworkTable.getTable("limelight").getDouble("tx", 0);
        ty = NetworkTable.getTable("limelight").getDouble("ty", 0);
        ta = NetworkTable.getTable("limelight").getDouble("ta", 0);
        angleToGoal = tx;//Math.toRadians(NetworkTable.getTable("GRIP/Target/Vision ID: " + (x + 1.0)).getNumber("Theta", -9999.0));
        calculateDistance();
        //SmartDashboard.putString("DriveToVisionTarget/A End of getReport", "We made it!");
        perpendicularAngleToGoal = 1.0;//Math.toRadians(NetworkTable.getTable("GRIP/Target/Vision ID: " + (x + 1.0)).getNumber("Height Ratio", -1.0));
        areaRatio = 1.0;//NetworkTable.getTable("GRIP/Target/Vision ID: " + (x + 1.0)).getNumber("Area Ratio", -1.0);
    }

    public boolean matchReportToTrack() {
        boolean updated = false;
        for(TrackedGoal goal : trackList) {
            if(goal.getScore(distanceToGoal, angleToGoal) > RobotMap.Vision.GOAL_SCORE_THRESHOLD) {
                updated = true;
                goal.update(distanceToGoal, perpendicularAngleToGoal, angleToGoal, areaRatio);
            }
        }
        return updated;
    }

    public void pruneDeadTracks() {
        for(int i = 0; i < trackList.size(); i++) {
            TrackedGoal goal = trackList.get(i);
            if(goal.isDead()) {
                if(trackList.get(i).equals(lastBestTrack)) {
                    lastBestTrack = null;
                }
                trackList.remove(i);
                i--;
            }
        }
    }

    public void selectBestTrack() {
        bestTrack = null; //ensures we don't return an old "best track"

        if(lastBestTrack != null && OI.getInstance().getVisionButton()) {
            bestTrack = lastBestTrack;
        } else {
            if(trackList.size() > 0) {
                bestTrack = trackList.get(0);   //Has a bestTrack if there is at least one track
            }
            for(TrackedGoal goal : trackList) {
                if(compareTracks(goal, bestTrack) == -1) {
                    bestTrack = goal; //Replaces bestTrack with higher-scoring track
                }
            }
        }
        lastBestTrack = bestTrack;
    }

    public int compareTracks(TrackedGoal goal1, TrackedGoal goal2) {
        if(goal1.getRankScore() > goal2.getRankScore()) {
            return -1;  //If goal 1 is better than goal 2, return -1
        } else if(goal1.getRankScore() < goal2.getRankScore()) {
            return 1;   //If goal 2 is better than goal 1, return 1
        } else {
            return 0; //If goal scores are equal, return 0
        }
    }

    public double getTargetRange() {
        if(bestTrack != null) {
            return bestTrack.getRange();
        } else {
            return -9999.0;
        }
    }

    public double getAngleToTargetInRadians() {
        return bestTrack.getAngleToTarget();
    }

    public double getTargetX() {
        return bestTrack.getX();
    }

    public double getAreaRatio() {
        return bestTrack.getAreaRatio();
    }

    public boolean hasBestTrack() {
        return bestTrack != null;
    }
    public double getNumOfTargets() {
        return numOfTargets;
    }
    public void calculateDistance() {
        final double L = (8.0 / 12.0);
        //160 is not correct and we cannot know until we find out specs of camera
        double w = 160.0, Wp = NetworkTable.getTable("limelight").getDouble("tshort", 0),
        cameraFOV = 52.0;
        distanceToGoal = (L * w) / (2.0 * Wp * Math.tan(Math.toRadians(cameraFOV / 2.0)));
        SmartDashboard.putNumber("Vision/tshort", Wp);
        SmartDashboard.putNumber("Vision/Distance To Goal", distanceToGoal);
    }
    public double getTx() {
        return tx;
    }
    public boolean hasTarget() {
        return tv == 1;
    }

    public void logToDashboard() {
        if(bestTrack != null) {
            SmartDashboard.putNumber("Vision/BestTrack/Range", bestTrack.getRange());
            SmartDashboard.putNumber("Vision/BestTrack/AngleToTarget", Math.toDegrees(bestTrack.getAngleToTarget()));
            SmartDashboard.putNumber("Vision/BestTrack/X", bestTrack.getX());
            SmartDashboard.putNumber("Vision/BestTrack/Y", bestTrack.getY());
            SmartDashboard.putNumber("Vision/BestTrack/Score", bestTrack.getScore(distanceToGoal, angleToGoal));
            SmartDashboard.putNumber("Vision/BestTrack/RankScore", bestTrack.getRankScore());
            SmartDashboard.putNumber("Vision/BestTrack/Target Angle", bestTrack.getAngle());
        } else {
            SmartDashboard.putNumber("Vision/BestTrack/Range", 0.0);
            SmartDashboard.putNumber("Vision/BestTrack/AngleToTarget", 0.0);
            SmartDashboard.putNumber("Vision/BestTrack/X", 0.0);
            SmartDashboard.putNumber("Vision/BestTrack/Y", 0.0);
            SmartDashboard.putNumber("Vision/BestTrack/Score", 0.0);
            SmartDashboard.putNumber("Vision/BestTrack/RankScore", 0.0);
            SmartDashboard.putNumber("Vision/BestTrack/Target Angle", -999.999);
        }

        SmartDashboard.putNumber("Vision/Number of Tracks", trackList.size());
        SmartDashboard.putNumber("Vision/Total Tracks Created", trackCounter);
        SmartDashboard.putNumber("Vision/Total Times Not Seeing Target", count);
        SmartDashboard.putBoolean("Vision/See Target", hasBestTrack());
        SmartDashboard.putNumber("Vision/tv", tv);
        SmartDashboard.putNumber("Vision/tx", tx);
        SmartDashboard.putNumber("Vision/ty", ty);
        SmartDashboard.putNumber("Vision/ta", ta);
    }
}