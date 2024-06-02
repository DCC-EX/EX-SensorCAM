# Development version for testing
## Introduction
This is a prerelease version which addresses some minor annoyances in screen formatting and should also eliminate the occasional eronious glitch from i2c timeouts.
Version 0.2.00+ address is no longer restricted to below ESP32CAP and is no longer compatible with IO_EXIOExpander devices but remains based on that software.

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




  
