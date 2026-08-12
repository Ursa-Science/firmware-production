Pump Module Refactor:

Current firmware:
Supports a dosing engine.

refactor target:
1. replace dosing engine with step counter.
-calibration will be done by the MIK by a user to fine tune ml/min rate
-When the MIK issues a command to start a dose or draw (this will be setup in the GUI under process controls), The MIK will convert dose/draw rate to stepper motor steps, and send this via PDO to the pump module.
- the pump module will receive the "steps to run" at the selected rate (by user setting up process) from the PDO from the MIK and then execute the process setup  on the MIK when signaled
- once started, the pump module will run the prescribed steps at the rate given by the MIK ,and keep a counter that tracks steps. Once the step counter reaches "0", the pump module will stop the pump.

prompt:
1. together, lets plan a modular, KISS, efficient approach to refactor the pump modules firmware to make this change
2. investigate what changes need to be made in the module's EDS file and firmware source files 