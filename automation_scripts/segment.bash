#!/usr/bin/env bash
# set -euo pipefail

cd results
rm -rf mesh_seg
mkdir mesh_seg
cd mesh_seg

isobin="/home/srg/Projects/isoperimetric_dijkstras/build/bin/iso_dij"
dbloc="/home/srg/Projects/databases/MeshsegBenchmark/data/obj"
mapfile -t files < <(find "$dbloc" -maxdepth 1 -type f -name "*.obj" | sort -V)

for z in "${files[@]}"; do
    this_id=$counter
    ((counter++))
    (
        mkdir $this_id
        cd $this_id

        timeout 300s "$isobin" "$z"
        tc=$?
        cd ..

        if [[ $tc -eq 124 ]]; then
            echo "  Timeout — removing $this_id"
            rm -rf "$this_id"
            echo "$z" >> timeout."$this_id"
        elif [[ $tc -eq 134 ]]; then
            echo "  Crash — removing $this_id"
            rm -rf "$this_id"
            echo "$z" >> crash."$this_id"
        elif [[ ! -s "./$this_id/curvenet.obj" ]]; then
            echo " Empty Curve Net"
            rm -rf "$this_id"
            echo "$z" >> emptycnet."$this_id"
        else
            cp "$z" ./"$this_id"
        fi

        echo "Finished $z (job $this_id)"
    )
done

# Wait for all background jobs to complete
wait

echo "All jobs complete."
