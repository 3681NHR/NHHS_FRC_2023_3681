package team3681.robot.lib.hardware.HID;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.lang.RuntimeException;
/**
 * Universal controller, read more below!
 * @see HIDWrapperBuilder
 */
import java.util.ArrayList;

public class HIDWrapper {

    private GenericHID controller;

    // NOTE: I hear using instanceof is kinda bad practice because it makes the code
    // not as modifiable and therefore not polymorphible(if that even is a word)
    // if someone wants they can fix it or like, yknow make better code
    // or hear me out, not use this at all and stick with directly creating
    // controller objects.
    // this is my attempt at better encapsulation.
    // - Daniel

    /**
     * Constructor for HIDWrapper. Use with the builder pattern.
     * 
     * @see HIDWrapperBuilder
     * @param controller
     */
    public HIDWrapper(GenericHID controller) {
        if (HIDSort(controller) != null) {
            this.controller = HIDSort(controller);
        } else {
            throw new RuntimeException("That is not a valid controller type.");
        }
    }

    private GenericHID HIDSort(GenericHID controller) {
        if (controller instanceof XboxController) {
            return (XboxController) controller;

        } else if (controller instanceof PS4Controller) {
            return controller = (PS4Controller) controller;

        } else if (controller instanceof Joystick) {
            return controller = (Joystick) controller;

        } else if (controller instanceof GenericHID) {
            return controller;

        } else {
            return null;
        }
    }

    /**
     * KEY FOR XBOX CONTROLLER INFO:
     * <p>
     * [0] - Left Trigger Axis
     * <p>
     * [1] - Right Trigger Axis
     * <p>
     * [2] - Left Joystick X
     * <p>
     * [3] - Left Joystick Y
     * <p>
     * [4] - Right Joystick X
     * <p>
     * [5] - Right Joystick Y
     * <p>
     * [6] - Back Button
     * <p>
     * [7] - Start Button
     * <p>
     * [8] - Left Bumper
     * <p>
     * [9] - Right Bumper
     * <p>
     * [10] - A Button (CONT)
     * <p>
     * [11] - B Button (CONT)
     * <p>
     * [12] - X Button (CONT)
     * <p>
     * [13] - Y Button (CONT)
     * <p>
     * [14] - A Button (SINGLE)
     * <p>
     * [15] - B Button (SINGLE)
     * <p>
     * [16] - X Button (SINGLE)
     * <p>
     * [17] - Y Button (SINGLE)
     * <p>
     * <p>
     * I am so sorry. Also feel free to add more information if you ever need it.
     * Doubt it though.
     * 
     * @return all info from the xboxcontroller
     */
    public double[] listenXB() {
        double[] xboxInfo = new double[18];
        if (controller instanceof XboxController) {
            XboxController xbox = (XboxController) controller;

            xboxInfo[0] = xbox.getLeftTriggerAxis();
            xboxInfo[1] = xbox.getRightTriggerAxis();

            xboxInfo[2] = xbox.getLeftX();
            xboxInfo[3] = xbox.getLeftY();

            xboxInfo[4] = xbox.getRightX();
            xboxInfo[5] = xbox.getRightY();

            // NOTE: BOOLEANS CONVERTED TO DOUBLE

            xboxInfo[6] = xbox.getBackButton() ? 1.0 : 0.0;
            xboxInfo[7] = xbox.getStartButton() ? 1.0 : 0.0;

            xboxInfo[8] = xbox.getLeftBumper() ? 1.0 : 0.0;
            xboxInfo[9] = xbox.getRightBumper() ? 1.0 : 0.0;

            xboxInfo[10] = xbox.getAButton() ? 1.0 : 0.0;
            xboxInfo[11] = xbox.getBButton() ? 1.0 : 0.0;
            xboxInfo[12] = xbox.getXButton() ? 1.0 : 0.0;
            xboxInfo[13] = xbox.getYButton() ? 1.0 : 0.0;

            xboxInfo[14] = xbox.getAButtonPressed() ? 1.0 : 0.0;
            xboxInfo[15] = xbox.getBButtonPressed() ? 1.0 : 0.0;
            xboxInfo[16] = xbox.getXButtonPressed() ? 1.0 : 0.0;
            xboxInfo[17] = xbox.getYButtonPressed() ? 1.0 : 0.0;

        } else {
            return null;
        }
        return xboxInfo;
    }

    /**
     * KEY FOR PS4 CONTROLLER INFO:
     * <p>
     * [0] - L2 Button
     * <p>
     * [1] - R2 Button
     * <p>
     * [2] - Left Joystick X
     * <p>
     * [3] - Left Joystick Y
     * <p>
     * [4] - Right Joystick X
     * <p>
     * [5] - Right Joystick Y
     * <p>
     * [6] - Share Button
     * <p>
     * [7] - Options Button
     * <p>
     * [8] - L1 Button
     * <p>
     * [9] - R1 Button
     * <p>
     * [10] - Triangle Button (CONT)
     * <p>
     * [11] - Circle Button (CONT)
     * <p>
     * [12] - Cross Button (CONT)
     * <p>
     * [13] - Square Button (CONT)
     * <p>
     * [14] - Triangle Button (SINGLE)
     * <p>
     * [15] - Circle Button (SINGLE)
     * <p>
     * [16] - Cross Button (SINGLE)
     * <p>
     * [17] - Square Button (SINGLE)
     * <p>
     * <p>
     * I am so sorry. Also feel free to add more information if you ever need it.
     * Doubt it though.
     * 
     * @return all info from the PS4 controller
     */
    public double[] listenPS4() {
        double[] ps4Info = new double[18];
        if (controller instanceof PS4Controller) {
            PS4Controller ps4 = (PS4Controller) controller;

            ps4Info[0] = ps4.getL2Axis();
            ps4Info[1] = ps4.getR2Axis();

            ps4Info[2] = ps4.getLeftX();
            ps4Info[3] = ps4.getLeftY();

            ps4Info[4] = ps4.getRightX();
            ps4Info[5] = ps4.getRightY();

            // NOTE: BOOLEANS CONVERTED TO DOUBLE

            ps4Info[6] = ps4.getShareButton() ? 1.0 : 0.0;
            ps4Info[7] = ps4.getOptionsButton() ? 1.0 : 0.0;

            ps4Info[8] = ps4.getL1Button() ? 1.0 : 0.0;
            ps4Info[9] = ps4.getR1Button() ? 1.0 : 0.0;

            ps4Info[10] = ps4.getTriangleButton() ? 1.0 : 0.0;
            ps4Info[11] = ps4.getCircleButton() ? 1.0 : 0.0;
            ps4Info[12] = ps4.getCrossButton() ? 1.0 : 0.0;
            ps4Info[13] = ps4.getSquareButton() ? 1.0 : 0.0;

            ps4Info[14] = ps4.getTriangleButtonPressed() ? 1.0 : 0.0;
            ps4Info[15] = ps4.getCircleButtonPressed() ? 1.0 : 0.0;
            ps4Info[16] = ps4.getCrossButtonPressed() ? 1.0 : 0.0;
            ps4Info[17] = ps4.getSquareButtonPressed() ? 1.0 : 0.0;

        } else {
            return null;
        }
        return ps4Info;
    }

    /** @unfinished
     * look at the other methods and finish later.
     * I also suggest actually seeing what is outputted. I dont know what all the methods do.
     * @deprecated
     * @return all info from the joystick
     */
    public double[] listenJS() {
        double[] joystickInfo = new double[18];
        if (controller instanceof Joystick) {
            Joystick js = (Joystick) controller;

            joystickInfo[0] = js.getThrottle();
        } else {
            return null;
        }
        return joystickInfo;
    }

}