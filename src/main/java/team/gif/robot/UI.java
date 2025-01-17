package team.gif.robot;

/**
 *   Initialize the shuffleboard here.
 *   Shuffleboard is a modern looking driveteam focused dashboard. It displays network
 *    tables data using a variety of widgets that can be positioned and controlled with robot code.
 *   Helpful link: https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/index.html

 *   Use the variable(shuffleboardTab) to add it to your shuffleboard.
 *   Example: shuffleboardTab.addBoolean("title of widget", Robot.arm::getPos());
 *   There is many more functions that you can use, example addString, addNumber, etc.
 */

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

public class UI {

    public UI() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("FRC 2025");
        ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");

        diagnosticsTab.addDouble("Swerve FL temp",Robot.swerveDrive::fLDriveTemp).withWidget(BuiltInWidgets.kTextView).withPosition(0,0);
        diagnosticsTab.addDouble("Swerve FR temp",Robot.swerveDrive::fRDriveTemp).withWidget(BuiltInWidgets.kTextView).withPosition(1,0);
        diagnosticsTab.addDouble("Swerve BL temp",Robot.swerveDrive::rLDriveTemp).withWidget(BuiltInWidgets.kTextView).withPosition(0,1);
        diagnosticsTab.addDouble("Swerve BR temp",Robot.swerveDrive::rRDriveTemp).withWidget(BuiltInWidgets.kTextView).withPosition(1,1);
    }
}
