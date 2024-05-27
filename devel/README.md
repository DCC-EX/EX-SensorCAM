# Development version for testing
## Introduction
This is a prerelease version which addresses some minor annoyances in screen formatting and should also eliminate the occasional eronious glitch from i2c timeouts.

## Requirements

Should work in conjunction with sensorCAM versions ABOVE 0.1.70 (not yet available)

Should ALSO work with earlier releases PROVIDED the user can reserve and preset two sensors in bank 0 for compatibility.
To get pre 0.1.70 sensorCAM's to work, reserve S06 and set to 0 using o06, and set another to 1 using, for example (lima) l07 say.
The two preset sensors can not be saved in eprom so need to be set each reboot.

### Please verify these fixes and report issues.

1.  No spurious momentary (~20 milliSecond)trips of sensors appearing on CS (but not on CAM).
2.  No problems with - < U R >, < Ur >, <Nr 0>, <NR 0>, <N R> (all identical but sometimes seeming not to re-reference ALL sensors.
     Note: old myFilter accepts U as alternative to < N >
3.	Satisfactory performance from <NT> <Nt 10> <Nt 44>  
	 Note: The multiple (repeat) line format <NT 10> (2 up to 30) should give from 2 to 30 repetitions with diff scores capped at 99
4.  all other commands working as documented/expected




  
