package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class Button implements Sendable {
    private final String name;
    private final Runnable onClick;

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
