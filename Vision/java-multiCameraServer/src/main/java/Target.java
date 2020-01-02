import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;

public class Target {
    private Point smallest0, smallest1, largest0, largest1, centerPoint;
    private double distance, angle, heightRatio, targetAngle, targetLength, number;
    public Target() {   
        setSmallest0(new Point(-1.0, -1.0));
        setSmallest1(new Point(-1.0, -1.0));
        setLargest0(new Point(-1.0, -1.0));
        setLargest1(new Point(-1.0, -1.0));
        setCenterPoint(new Point(-1.0, -1.0));
        setDistance(-999.999);
        setAngle(-999.999);
        setHeightRatio(-999.999);
        setTargetAngle(-999.999);
        setTargetLength(-999.999);
    }

    /**
     * @return the number
     */
    public double getNumber() {
        return number;
    }

    /**
     * @param number the number to set
     */
    public void setNumber(double number) {
        this.number = number;
    }

    public void publishToNetworkTable()
    { 
        NetworkTable.getTable("GRIP/Target").putNumber("Vision ID: ", getNumber());
        NetworkTable.getTable("GRIP/Target/Vision ID: " + getNumber()).putNumber("Center X", centerPoint.x);
        NetworkTable.getTable("GRIP/Target/Vision ID: " + getNumber()).putNumber("Center Y", centerPoint.y);
        NetworkTable.getTable("GRIP/Target/Vision ID: " + getNumber()).putNumber("Distance", distance);
        NetworkTable.getTable("GRIP/Target/Vision ID: " + getNumber()).putNumber("Theta", angle);
        NetworkTable.getTable("GRIP/Target/Vision ID: " + getNumber()).putNumber("Height Ratio", heightRatio);
        NetworkTable.getTable("GRIP/Target/Vision ID: " + getNumber()).putNumber("Target Angle", targetAngle);
        NetworkTable.getTable("GRIP/Target/Vision ID: " + getNumber()).putNumber("Target Length", targetLength);
    }

    /**
     * @return the targetLength
     */
    public double getTargetLength() {
        return targetLength;
    }

    /**
     * @param targetLength the targetLength to set
     */
    public void setTargetLength(double targetLength) {
        this.targetLength = targetLength;
    }

    /**
     * @return the targetAngle
     */
    public double getTargetAngle() {
        return targetAngle;
    }

    /**
     * @param targetAngle the targetAngle to set
     */
    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    /**
     * @return the heightRatio
     */
    public double getHeightRatio() {
        return heightRatio;
    }

    /**
     * @param heightRatio the heightRatio to set
     */
    public void setHeightRatio(double heightRatio) {
        this.heightRatio = heightRatio;
    }

    /**
     * @return the angle
     */
    public double getAngle() {
        return angle;
    }

    /**
     * @param angle the angle to set
     */
    public void setAngle(double angle) {
        this.angle = angle;
    }

    /**
     * @return the distance
     */
    public double getDistance() {
        return distance;
    }

    /**
     * @param distance the distance to set
     */
    public void setDistance(double distance) {
        this.distance = distance;
    }

    /**
     * @return the centerPoint
     */
    public Point getCenterPoint() {
        return centerPoint;
    }

    /**
     * @param centerPoint the centerPoint to set
     */
    public void setCenterPoint(Point centerPoint) {
        this.centerPoint = centerPoint;
    }

    /**
     * @return the largest1
     */
    public Point getLargest1() {
        return largest1;
    }

    /**
     * @param largest1 the largest1 to set
     */
    public void setLargest1(Point largest1) {
        this.largest1 = largest1;
    }

    /**
     * @return the largest0
     */
    public Point getLargest0() {
        return largest0;
    }

    /**
     * @param largest0 the largest0 to set
     */
    public void setLargest0(Point largest0) {
        this.largest0 = largest0;
    }

    /**
     * @return the smallest1
     */
    public Point getSmallest1() {
        return smallest1;
    }

    /**
     * @param smallest1 the smallest1 to set
     */
    public void setSmallest1(Point smallest1) {
        this.smallest1 = smallest1;
    }

    /**
     * @return the smallest0
     */
    public Point getSmallest0() {
        return smallest0;
    }

    /**
     * @param smallest0 the smallest0 to set
     */
    public void setSmallest0(Point smallest0) {
        this.smallest0 = smallest0;
    }
}