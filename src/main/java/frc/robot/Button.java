package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Represents a clickable button in Shuffleboard
 */
public class Button implements Sendable {
    private final String name;
    private final Runnable onClick;

    /**
     * Creates a new Button with a name and a click event
     * @param name The name/label of the button
     * @param onClick The runnable to run when the button is clicked
     */
    public Button(String name, Runnable onClick) {
        this.name = name;
        this.onClick = onClick;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("button");
        builder.addStringProperty("name", () -> name, null);
        builder.addIntegerProperty("counter", () -> 0, newValue -> onClick.run());
    }
}
