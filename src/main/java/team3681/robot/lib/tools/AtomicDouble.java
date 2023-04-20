package team3681.robot.lib.tools;

import java.util.concurrent.atomic.AtomicLong;

public class AtomicDouble {
    private AtomicLong bits;

    public AtomicDouble(double value) {
        bits = new AtomicLong(Double.doubleToLongBits(value));
    }

    public void set(double value) {
        bits.set(Double.doubleToLongBits(value));
    }

    public double get() {
        return Double.longBitsToDouble(bits.get());
    }

    public double getAndAdd(double delta) {
        long oldBits;
        long newBits;
        do {
            oldBits = bits.get();
            double oldValue = Double.longBitsToDouble(oldBits);
            double newValue = oldValue + delta;
            newBits = Double.doubleToLongBits(newValue);
        } while (!bits.compareAndSet(oldBits, newBits));
        return Double.longBitsToDouble(oldBits);
    }
}

