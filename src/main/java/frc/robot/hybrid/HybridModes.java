package frc.robot.hybrid;

import java.util.HashMap;

public class HybridModes {
    private HashMap<String, ControlVector> modes = new HashMap<>();

    public void addMode(String name, ControlVector mode) {
        modes.put(name, mode);
    }

    /**
     *
     * @param name
     * @return
     */
    public ControlVector getMode(String name) {
        return modes.get(name);
    }
}
