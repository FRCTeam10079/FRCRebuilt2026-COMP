PS C:\Comp Code For Auburn\FRCRebuilt2026-COMP>  ${env:HALSIM_EXTENSIONS}=''; ${env:PATH}='C:\Comp Code For Auburn\FRCRebuilt2026-COMP\build\jni\release;C:\Windows\system32\'; & 'C:\Users\Public\wpilib\2026\jdk\bin\java.exe' '-agentlib:jdwp=transport=dt_socket,server=n,suspend=y,address=localhost:52012' '@C:\Users\robot\AppData\Local\Temp\cp_a5jst2o4g47b12ujhx7xyl7sm.argfile' 'frc.robot.Main' 
HAL Extensions: No extensions found
********** Robot program starting **********
NT: could not open persistent file 'networktables.json': no such file or directory (this can be ignored if you aren't expecting persistent values)
NT: Listening on NT3 port 1735, NT4 port 5810
No log provided with the AKIT_LOG_PATH environment variable or through AdvantageScope. Enter path to file: C:\Comp Code For Auburn\FRCRebuilt2026-COMP\test1.wpilog
Error at org.littletonrobotics.junction.wpilog.WPILOGReader.start(WPILOGReader.java:58): [AdvantageKit] The replay log was not produced by AdvantageKit.
        at org.littletonrobotics.junction.wpilog.WPILOGReader.start(WPILOGReader.java:58)
        at org.littletonrobotics.junction.Logger.start(Logger.java:188)
        at frc.robot.Robot.<init>(Robot.java:69)
        at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:387)
        at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:527)
        at frc.robot.Main.main(Main.java:14)

[AdvantageKit] Logging to "C:\Comp Code For Auburn\FRCRebuilt2026-COMP\test1_sim.wpilog"
[AdvantageKit] Log sent to AdvantageScope.
PS C:\Comp Code For Auburn\FRCRebuilt2026-COMP> ^C                                                  
PS C:\Comp Code For Auburn\FRCRebuilt2026-COMP>                                                     
PS C:\Comp Code For Auburn\FRCRebuilt2026-COMP>  c:; cd 'c:\Comp Code For Auburn\FRCRebuilt2026-COMP'; ${env:HALSIM_EXTENSIONS}=''; ${env:PATH}='C:\Comp Code For Auburn\FRCRebuilt2026-COMP\build\jni\release;C:\Windows\system32\'; & 'C:\Users\Public\wpilib\2026\jdk\bin\java.exe' '-agentlib:jdwp=transport=dt_socket,server=n,suspend=y,address=localhost:52073' '@C:\Users\robot\AppData\Local\Temp\cp_a5jst2o4g47b12ujhx7xyl7sm.argfile' 'frc.robot.Main'
HAL Extensions: No extensions found
********** Robot program starting **********
NT: Listening on NT3 port 1735, NT4 port 5810
No log provided with the AKIT_LOG_PATH environment variable or through AdvantageScope. Enter path to file: C:\Comp Code For Auburn\FRCRebuilt2026-COMP\FRC_20260401_024553.wpilog
Error at org.littletonrobotics.junction.wpilog.WPILOGReader.start(WPILOGReader.java:58): [AdvantageKit] The replay log was not produced by AdvantageKit.
        at org.littletonrobotics.junction.wpilog.WPILOGReader.start(WPILOGReader.java:58)
        at org.littletonrobotics.junction.Logger.start(Logger.java:188)
        at frc.robot.Robot.<init>(Robot.java:69)
        at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:387)
        at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:527)
        at frc.robot.Main.main(Main.java:14)

[AdvantageKit] Logging to "C:\Comp Code For Auburn\FRCRebuilt2026-COMP\FRC_20260401_024553_sim.wpilog"
[AdvantageKit] Log sent to AdvantageScope.

---