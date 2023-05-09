package team3681.robot.lib.hardware.HID;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Joystick;

/**.<p>
 * Builds HIDWrapper and allows for flexibility/encapsulation.
 * <p>
 * Why?: I anticipate the need to have different controllers in the future,
 * <p>
 * and this allows me or you to not need to directly create the objects
 * <P>
 * of the required class.
 * <p>
 * ===== example =====
 * <p>
 * HIDWrapper xboxWrapper = new HIDWrapperBuilder()
 * <p>
 * .withXboxController(0)
 * <p>
 * .build();
 * <p>
 * -
 * <p>
 * HIDWrapper ps4Wrapper = new HIDWrapperBuilder()
 * <p>
 * .withPS4Controller(0)
 * <p>
 * .build();
 * <p>
 * -
 * <p>
 * HIDWrapper joystickWrapper = new HIDWrapperBuilder()
 * <p>
 * .withJoystick(0)
 * <p>
 * .build();
 *
 * @see HIDWrapper
 * @see GenericHID
 * @see XboxController
 * @see PS4Controller
 * @see Joystick
 * @deprecated I dont appreciate how I made this, maybe you will.
 * 
 * @author Me.
 */
public class HIDWrapperBuilder {
    //NOTE: this makes sense.
    private GenericHID controller;

    public HIDWrapperBuilder withXboxController(int port) {
        controller = new XboxController(port);
        return this;
    }

    public HIDWrapperBuilder withPS4Controller(int port) {
        controller = new PS4Controller(port);
        return this;
    }

    public HIDWrapperBuilder withJoystick(int port) {
        controller = new Joystick(port);
        return this;
    }

    public HIDWrapperBuilder withGenericHID(int port) {
        controller = new GenericHID(port);
        return this;
    }

    /**
     * HIDWrapper constructor
     * 
     * @return
     */
    public HIDWrapper build() {
        return new HIDWrapper(controller);
    }
}
