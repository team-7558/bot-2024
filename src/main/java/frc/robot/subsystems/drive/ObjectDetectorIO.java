// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ObjectDetectorIO {
    @AutoLog
    public static class ObjectDetectorIOInputs {
        public boolean connected = false;
        public boolean tv = false;
        public double tx = 0.0;
        public double ty = 0.0;
        public double ta = 0.0;
    }

    public default void updateInputs(ObjectDetectorIOInputs inputs){}

    public default void setPipeline(int id){}

}
