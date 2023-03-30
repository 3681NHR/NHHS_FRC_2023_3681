package team3681.lib.behavior.interfaces;

public interface inputManager {
     /**
     * Method to add an input device to the manager.
     * @param device the input device to add.
     */
    public void addInputDevice(Object device);
    
    /**
     * Method to remove an input device from the manager.
     * @param device the input device to remove.
     */
    public void removeInputDevice(Object device);

     /** Method to update the input manager and get the latest input values.
     */
    public void update();
}

