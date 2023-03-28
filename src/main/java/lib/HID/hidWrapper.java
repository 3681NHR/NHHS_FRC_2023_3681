// package lib.HID;

// import java.util.List;
// import java.util.Arrays;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.PS4Controller;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.Joystick;

// public class hidWrapper {

//     public List<Object> hidDevice;

//     public List<XboxController> xboxDevices;
//     public List<PS4Controller> ps4Devices;

//     public List<Joystick> joystickDevices;
//     public List<GenericHID> genericDevices;

//     //NOTE: Is this okay? Probably. I'm only using these to grab a class.
//     private static final XboxController XC = null;
//     private static final PS4Controller P4 = null;
//     private static final GenericHID GH = null;
//     private static final Joystick JS = null;

//     Class xboxClass = XC.getClass();
//     Class ps4Class = P4.getClass();
//     Class genericHIDClass = GH.getClass();
//     Class joystickClass = JS.getClass();
    
//     /** the HID wrapper takes an indefinite amount of these four objects and it sorts them into their own lists.
//      * @param XboxController 
//      * @param PS4Controller
//      * @param GenericHID
//      * @param Joystick
//      */
//     public hidWrapper(Object... hidArray) {
//         hidDevice = Arrays.asList(hidArray);
        
//     }
// /**Use once only, runs a for loop to sort HID then allows you to use them */
//     public void hidCalibrate(){
//         sort(); //NOTE: FOR LOOP
//     }

//     private void sort() {
//         Class hidClass;

//         Object hidObj;

//         if(hidDevice.isEmpty()) {return;} else {
//             for (int i = 0; i < hidDevice.size(); i++) {
//                 if (hidDevice.get(i) instanceof Object){
//                     hidObj = hidDevice.get(i);
//                     hidClass = hidObj.getClass();
//                     //NOTE: if hidObj is the same class, then add the thing after casting it to the lists and then return
//                            if (hidDevice.get(i).equals(xboxClass)){
//                             xboxDevices.add(XboxController.class.cast(hidObj));
//                     } else if (hidDevice.get(i).equals(ps4Class)) {
//                             ps4Devices.add(PS4Controller.class.cast(hidObj));
//                     } else if (hidDevice.get(i).equals(genericHIDClass)) {
//                             genericDevices.add(GenericHID.class.cast(hidObj));
//                     } else if (hidDevice.get(i).equals(joystickClass)) {
//                             joystickDevices.add(Joystick.class.cast(hidObj));
//                     } else {
//                         return;
//                     }
//                 } else {
//                     return;
//                 }
//             }
//             return;

//         }
//     }

//     public void XboxManager(String ControllerName) {
//         if (xboxDevices.isEmpty()) {return;} else {
//         }
//     }

//     class XboxActions {
//         //this class is for above
//         public boolean LBactive(XboxController controller) {
//             return controller.getLeftBumper();
//         }

//         public boolean RBactive(XboxController controller) {
//             return controller.getRightBumper();
//         }

//         public boolean LBactiveSingle(XboxController controller) {
//             return controller.getLeftBumperPressed();
//         }

//         public boolean RBactiveSingle(XboxController controller) {
//             return controller.getRightBumperPressed();
//         }

//         public boolean LBpressed(XboxController controller) {
//             return controller.getLeftBumperPressed();
//         }

//         public boolean RBpressed(XboxController controller) {
//             return controller.getRightBumperPressed();
//         }

//         public boolean A(XboxController controller) {
//             return controller.getAButton();
//         }

//         public boolean X(XboxController controller) {
//             return controller.getXButton();
//         }

//         public boolean Y(XboxController controller) {
//             return controller.getYButton();
//         }

//         public boolean B(XboxController controller) {
//             return controller.getBButton();
//         }

//         public boolean Asingle(XboxController controller) {
//             return controller.getAButtonPressed();
//         }

//         public boolean Xsingle(XboxController controller) {
//             return controller.getXButtonPressed();
//         }

//         public boolean Ysingle(XboxController controller) {
//             return controller.getYButtonPressed();
//         }

//         public boolean Bsingle(XboxController controller) {
//             return controller.getBButtonPressed();
//         }

//         public double leftJoyX(XboxController controller) {
//             return controller.getLeftX();
//         }

//         public double leftJoyY(XboxController controller) {
//             return controller.getLeftY();
//         }

//         public double rightJoyX(XboxController controller) {
//             return controller.getRightX();
//         }

//         public double rightJoyY(XboxController controller) {
//             return controller.getRightY();
//         }

//         public boolean homeButton(XboxController controller) {
//             return controller.getStartButton();
//         }
//     }
    
// }
