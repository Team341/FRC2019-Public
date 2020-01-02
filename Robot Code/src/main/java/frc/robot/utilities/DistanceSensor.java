package frc.robot.utilities;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class DistanceSensor {
    // A MB1013 distance sensor - http://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf
    // (pins 3, 6 and 7 from sensor to analog input 0)
    private AnalogInput sensor;

    public DistanceSensor(int port) {
      sensor = new AnalogInput(port);
    }
    
    // TODO - You will need to determine how to convert voltage to distance
    // (use information from the data sheet, or your own measurements)

    public double getVoltage() {
      return sensor.getVoltage();
    }
    
    public double getDistanceInCM() {
      return getVoltage() * RobotMap.Hab.MEASURED_VOLTAGE_TO_CM;
    }
    
    public void logToDashboard() {
      SmartDashboard.putNumber("Hab/DistanceToGround (volts)", getVoltage());
      SmartDashboard.putNumber("Hab/DistanceToGround (CM)", getDistanceInCM());
    }
  }