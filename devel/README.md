# Development version for testing
## Introduction
This is a prerelease version which addresses some minor annoyances in screen formatting and should also eliminate the occasional eronious glitch from i2c timeouts.  This Version 2.1 is to be used with ver 0.2.1 of sensorCAM.ino.  Earlier versions of sensorCAM.ino require the latching low (o06) of S06 For compatibility.  sensorCAM 0.2.1 now has two reserved sensors S00 and S06.  S06 may be used for "Twinning" only.
Version 0.2.0+ i2c address is no longer restricted to below ESP32CAP and is no longer interchangeable with IO_EXIOExpander.h but remains based on that software.  
The latest myFilter.h has an added command \<N b \$\> that implements the CAM's ( b $ ) block status command with v2.1 of IO_EXSensorCAM

## Requirements

new IO_EXSensorCAM.h should work in conjunction with sensorCAM versions ABOVE 0.1.70 (version 0.2.01 included)

sensorCAM.pde version 2.00+ has an addition to immediately mark (box) a new sensor on the CAM image in Processing 4.  It will work with all versions of sensorCAM.ino.  

### Please verify these fixes and report issues.

1.  No spurious momentary (~20 milliSecond) trips of sensors appearing on CS (but not on CAM).
2.  No problems with Native CS comands \<U R>, \<Nr>, \<Nr 0>, \<NR 0> (all identical but sometimes seeming not to re-reference ALL sensors.
     Note: old myFilter accepts \<U> as alternative to \<N>
3.	Satisfactory performance from \<NT> \<Nt 10> \<Nt 44>  
	 Note: The multiple (repeat) line format \<NT 10> (2-30) should give from 2 to 30 repetitions with diff scores capped at 99
4.  all other commands working as documented/expected




  
