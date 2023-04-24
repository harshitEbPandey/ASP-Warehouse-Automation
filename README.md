# ASP-Warehouse-Automation
ASP-Warehouse-Automation for robot path planning under expert constraints.

Requirements - 
Execution of the program needs clingo setup.
Command provided is for UNIX based systems, 
for WIN systems ./clingo.exe needs to be used.

The code can be run using the following command on shell prompt - 
clingo solution.asp ./simpleInstances/inst{x}.asp -c n={y}

Where x and y are user defined inputs, x defines the sample instance to be tested and y defines the maximum timestep mark for a run.

Results : 
Test# #Robots #Actions Sat@n CPU Time
--------------------------------------
  1      2 	19 	10 	0.462
  2 	 2 	17 	11 	0.932
  3 	 2 	10 	6 	0.040
  4 	 2 	10 	5 	0.043
  5 	 2 	10 	6 	0.056
--------------------------------------
