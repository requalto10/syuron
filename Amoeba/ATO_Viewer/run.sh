#!/bin/bash
venv_path="../ato-py/ATO_v3_venv/"
if [ -f $venv_path/bin/activate ]; then
    source ../ato-py/ATO_v3_venv/bin/activate
fi
datname=evolution_U_grid_36_v2_02_0
#plate=""
plate="--plate-data data/evolution_C_grid_36_v2_02_0.dat"
step=""
#step="--step 50"
#debug="-m pdb"
debug=""
view="--view 30 30"
position_map=grid_eval_B_layout.json
python3 $debug ATO_viewer.py data/${datname}.dat  --output movies/${datname}.mp4 ${step} ${view} $plate --position-map grid_eval_B_layout.json

