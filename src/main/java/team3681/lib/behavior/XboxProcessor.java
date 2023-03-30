package team3681.lib.behavior;

import team3681.lib.behavior.interfaces.inputManager;

import edu.wpi.first.wpilibj.XboxController;

import java.util.ArrayList;
import java.util.List;

public class XboxProcessor implements inputManager{

    List<XboxController> inputDevices = new ArrayList<>();

    @Override
    public void addInputDevice(Object device) {
        XboxController controller = (XboxController) device;
        inputDevices.add(controller);
        
    }

    @Override
    public void removeInputDevice(Object device) {
        XboxController controller = (XboxController) device;
        if (inputDevices.contains(controller)) {
            inputDevices.remove(controller);
        } else {
            return;
        }    
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void getList() {
        // TODO Auto-generated method stub
        
    };

}
