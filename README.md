# remoteid-simulator

Requirements:
ardupilot installed and merged with files for remoteid support, dronekit installed

Clone location:
clone into the same directory that contains ardupilot

Run using command:
multidrone_script.py -n 4

----------------------------------------------------------------------------------------

To test broadcasting and remoteID:
**Note: run all in seperate terminals**

run from broadcast directory:
./broadcast

launch code in vsc for:
./build/sitl/bin/arducopter -S -I0 --model + --speedup 1 --remoteid 616263646566303132333435363738397778797A --home 30.270083225404864,-97.7730248064704,0,180 --defaults ./Tools/autotest/default_params/copter.parm

run:
./build/sitl/bin/arducopter -S -I1 --model + --speedup 1 --remoteid 616263646566303132333435363738397778797B --home 30.270083225404864,-97.76852481,0,225 --defaults ./Tools/autotest/default_params/copter.parm

run:
python3 mult_drones.py -n 2
