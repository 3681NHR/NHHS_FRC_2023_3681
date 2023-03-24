package lib.interfaces;

public interface Act { //abstraction is real
    
    /* double num is there if you want it. No need to use it*/
    void action(double num); // what is your action 

    boolean finished(); //get if stopped

    void stop(); // stop it
}
