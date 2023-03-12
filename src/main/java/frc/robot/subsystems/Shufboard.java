// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 
 */
public class Shufboard extends SubsystemBase {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    //END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS'

    // The following definitions are for the positions for placing objects on the
    // field
    private boolean Position1 = false; // Top left if facing the field elements
    private boolean Position2 = false; // Bottom left if facing the field elements
    private boolean Position3 = false; // Top Middle if facing the field elements
    private boolean Position4 = false; // Bottom Middle if facing the field elements
    private boolean Position5 = false; // Top Right if facing the field elements
    private boolean Position6 = false; // Bottom Right if facing the field elements

    private final String driverTabName = "Driver Tab";

    public enum Position {
        LeftTop, LeftBottom, MiddleTop, MiddleBottom, RightTop, RightBottom, none
    };

    public Position SelectedPosition = Position.none;

    private ShuffleboardTab driverTab;

    /**
    *
    */
    public Shufboard() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // This is the constructor for our ShufBoard Class

        // This is where we will call routines to define screen(s)
        driverTab = Shuffleboard.getTab(this.driverTabName);

        // Shuffleboard.selectTab(this.driverTabName);
        addPositions(0, 0);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        setSelectedPosition();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private void addPositions(int x, int y) {
        // DriverTab.add("Position01",Position1);
        addPosition("Left Tag", Position1, x, y);
        addPosition("Left Offset", Position2, x, y + 1);
        addPosition("Mid Tag", Position3, x + 1, y);
        addPosition("Mid Offset", Position4, x + 1, y + 1);
        addPosition("Right Tag", Position5, x + 2, y);
        addPosition("Right Offset", Position6, x + 2, y + 1);
    }

    private void addPosition(String title, boolean value, Integer xpos, Integer ypos) {
        driverTab.add(title, value).withSize(1, 1).withPosition(xpos, ypos).withWidget("Toggle Button");
    }

    private void setSelectedPosition() {
        NetworkTableInstance tblInst = NetworkTableInstance.getDefault();
        NetworkTable tblShuffleBoard = tblInst.getTable("Shuffleboard");

        boolean p1 = tblShuffleBoard.getEntry("Driver Tab/Left Tag").getBoolean(false);
        boolean p2 = tblShuffleBoard.getEntry("Driver Tab/Left Offset").getBoolean(false);
        boolean p3 = tblShuffleBoard.getEntry("Driver Tab/Mid Tag").getBoolean(false);
        boolean p4 = tblShuffleBoard.getEntry("Driver Tab/Mid Offset").getBoolean(false);
        boolean p5 = tblShuffleBoard.getEntry("Driver Tab/Right Tag").getBoolean(false);
        boolean p6 = tblShuffleBoard.getEntry("Driver Tab/Right Offset").getBoolean(false);

        if (p1)
            SelectedPosition = Position.LeftTop;
        else if (p2)
            SelectedPosition = Position.LeftBottom;
        else if (p3)
            SelectedPosition = Position.MiddleTop;
        else if (p4)
            SelectedPosition = Position.MiddleBottom;
        else if (p5)
            SelectedPosition = Position.RightTop;
        else if (p6)
            SelectedPosition = Position.RightBottom;
        else
            SelectedPosition = Position.none;

    }
}