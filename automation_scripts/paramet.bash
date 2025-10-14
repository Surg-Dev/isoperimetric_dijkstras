#!/usr/bin/env bash
# set -euo pipefail

cd results
rm -rf paramet
mkdir paramet
cd paramet

isobin="/home/srg/Projects/isoperimetric_dijkstras/build/bin/iso_dij"
dbloc="/home/srg/Projects/databases/Parametrization/Obj_Files/Full"
max_jobs=60
counter=1
# touch failed_files

for z in "$dbloc"/*; do
    this_id=$counter
    ((counter++))

    while (( $(jobs | wc -l) >= max_jobs )); do
        sleep 0.5
    done

    (
        mkdir $this_id
        cd $this_id

        timeout 120s "$isobin" "$z"
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
        else 
            cp "$z" ./"$this_id"
        fi

        echo "Finished $z (job $this_id)"
    ) &
done

# Wait for all background jobs to complete
wait

echo "All jobs complete."
