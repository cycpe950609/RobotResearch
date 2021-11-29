#!/bin/sh
rostopic echo -b $1 -p /LaserScanSim > LaserScanSim.csv
rostopic echo -b $1 -p /ptn_dir > ptn_dir.csv
rostopic echo -b $1 -p /_last_direction > last_direction.csv
rostopic echo -b $1 -p /base_idx > base_idx.csv
rostopic echo -b $1 -p /last_new_index > last_new_index.csv
rostopic echo -b $1 -p /degree_pt0_ptnew > degree_pt0_ptnew.csv 
rostopic echo -b $1 -p /LastUpdateDirection > LastUpdateDirection.csv 
rostopic echo -b $1 -p /LastDirection > LastDirection.csv 
rostopic echo -b $1 -p /NewDirection > NewDirection.csv 
