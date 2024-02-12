package frc.robot;

import edu.wpi.first.math.MathUtil;

public class JoystickMath {
    // note: minPower and percentJoystickisMaxSpeed are in range [0, 1]
    public static double convert(double joystickSpeed, double exponent, double minPower, double percentJoystickIsMaxSpeed) {
        if (Math.abs(joystickSpeed) < constants.kControllerDeadband) return 0;
        return MathUtil.clamp(-constants.kMaxSpeed*(minPower+(1-minPower)*Math.pow(((Math.abs(joystickSpeed)-constants.kControllerDeadband)/(percentJoystickIsMaxSpeed-constants.kControllerDeadband)), exponent))*Math.signum(joystickSpeed), -constants.kMaxSpeed, constants.kMaxSpeed);
    }
}
