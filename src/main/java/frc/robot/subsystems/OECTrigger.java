package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;
import java.util.function.BooleanSupplier;

public class OECTrigger extends Trigger {
    private final BooleanSupplier m_condition;
    private final EventLoop m_loop;

    public OECTrigger(EventLoop loop, BooleanSupplier condition) {
        super(loop, condition);
        m_loop = requireNonNullParam(loop, "loop", "OECTrigger");
        m_condition = requireNonNullParam(condition, "condition", "OECTrigger");
    }

    public OECTrigger(BooleanSupplier condition) {
        super(condition);
        m_loop = requireNonNullParam(CommandScheduler.getInstance().getDefaultButtonLoop(), "loop", "OECTrigger");
        m_condition = requireNonNullParam(condition, "condition", "OECTrigger");
    }

    public OECTrigger everyTimeItsTrue(Command command) {
        requireNonNullParam(command, "command", "everyTimeItsTrue");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();

                    if (pressed) {
                        command.schedule();
                    }

                    m_pressedLast = pressed;
                }
        });
        return this;
    }

    public OECTrigger everyTimeItsFalse(Command command) {
        requireNonNullParam(command, "command", "everyTimeItsFalse");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();

                    if (!pressed) {
                        command.schedule();
                    }

                    m_pressedLast = pressed;
                }
        });
        return this;
    }

    // Overriding all methods below so they return OECTrigger instead of Trigger

    @Override
    public OECTrigger onTrue(Command command) {
        requireNonNullParam(command, "command", "onTrue");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();

                    if (!m_pressedLast && pressed) {
                        command.schedule();
                    }

                    m_pressedLast = pressed;
                }
        });
        return this;
    }

    @Override
    public OECTrigger onFalse(Command command) {
        requireNonNullParam(command, "command", "onFalse");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();

                    if (m_pressedLast && !pressed) {
                        command.schedule();
                    }

                    m_pressedLast = pressed;
                }
        });
        return this;
    }

    @Override
    public OECTrigger whileTrue(Command command) {
        requireNonNullParam(command, "command", "whileTrue");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();

                    if (!m_pressedLast && pressed) {
                        command.schedule();
                    } else if (m_pressedLast && !pressed) {
                        command.cancel();
                    }

                    m_pressedLast = pressed;
                }
        });
        return this;
    }

    @Override
    public OECTrigger whileFalse(Command command) {
        requireNonNullParam(command, "command", "whileFalse");
            m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

            @Override
            public void run() {
                boolean pressed = m_condition.getAsBoolean();

                if (m_pressedLast && !pressed) {
                    command.schedule();
                } else if (!m_pressedLast && pressed) {
                    command.cancel();
                }

                m_pressedLast = pressed;
            }
        });
        return this;
    }

    @Override
    public OECTrigger toggleOnTrue(Command command) {
        requireNonNullParam(command, "command", "toggleOnTrue");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();

                    if (!m_pressedLast && pressed) {
                        if (command.isScheduled()) {
                            command.cancel();
                        } else {
                            command.schedule();
                        }
                    }

                    m_pressedLast = pressed;
                }
        });
        return this;
    }

    @Override
    public OECTrigger toggleOnFalse(Command command) {
        requireNonNullParam(command, "command", "toggleOnFalse");
        m_loop.bind(
            new Runnable() {
                private boolean m_pressedLast = m_condition.getAsBoolean();

                @Override
                public void run() {
                    boolean pressed = m_condition.getAsBoolean();

                    if (m_pressedLast && !pressed) {
                        if (command.isScheduled()) {
                            command.cancel();
                        } else {
                            command.schedule();
                        }
                    }

                    m_pressedLast = pressed;
                }
        });
        return this;
    }

    @Override
    public OECTrigger and(BooleanSupplier trigger) {
        return new OECTrigger(() -> m_condition.getAsBoolean() && trigger.getAsBoolean());
    }

    @Override
    public OECTrigger or(BooleanSupplier trigger) {
        return new OECTrigger(() -> m_condition.getAsBoolean() || trigger.getAsBoolean());
    }

    @Override
    public OECTrigger negate() {
        return new OECTrigger(() -> !m_condition.getAsBoolean());
    }

    @Override
    public OECTrigger debounce(double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    @Override
    public OECTrigger debounce(double seconds, Debouncer.DebounceType type) {
        return new OECTrigger(
            new BooleanSupplier() {
                final Debouncer m_debouncer = new Debouncer(seconds, type);

                @Override
                public boolean getAsBoolean() {
                    return m_debouncer.calculate(m_condition.getAsBoolean());
                }
            });
    }
}
