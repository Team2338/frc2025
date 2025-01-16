package team.gif.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UiSmartDashboard {

    /**
     *  Widgets (e.g. gyro),
     *  buttons (e.g. SmartDashboard.putData("Reset", new ResetHeading()); ),
     *  and Chooser options (e.g. auto mode)
     *
     *  Placed on a user defined dashboard tab
     *  After SmartDashboard loads for the first time, move items from network table onto Dashboard tab
     *  and save file as "YYYY shuffleboard layout.json"
     */
    public UiSmartDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard"); // Gets a reference to the shuffleboard tab

        // place input boxes on SmartDashboard
        SmartDashboard.putNumber("Shoot %",0);
        SmartDashboard.putNumber("Index %",0);
    }

    /**
     * Values which are updated periodically should be placed here
     *
     * Convenient way to format a number is to use putString w/ format:
     *     SmartDashboard.putString("Elevator", String.format("%11.2f", Elevator.getPosition()));
     */
    public void updateUI() {
        // test to see if we can get a number from the dashboard
        boolean state = SmartDashboard.getNumber("Shoot %", 0) > .5;
        SmartDashboard.putBoolean(">0.5", state); // display red/green based on value on screen
    }
}
