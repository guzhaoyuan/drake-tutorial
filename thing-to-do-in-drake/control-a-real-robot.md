# Control Real KUKA arm

Data is transmitted through LCM in Drake. To controller a real robot, control signal from Drake should be translated from LCM to robot language, which could be EtherCat, Serial, or PWM signals, etc. The translator is the driver we should have implement for each robot we want to drive.

For example, if we want to drive a KUKA arm, we would need to translate LCM to KUKA Robot Language \(KRL\).

