#!/usr/bin/env bash
# set -euo pipefail

cd results
rm -rf mpz13
mkdir mpz13
cd mpz13

isobin="/home/srg/Projects/isoperimetric_dijkstras/build/bin/iso_dij"
dbloc="/home/srg/Projects/databases/MPZ14/models/inputmodels"
max_jobs=32
counter=1
# touch failed_files

for z in "$dbloc"/*.obj; do
    this_id=$counter
    ((counter++))

    while (( $(jobs | wc -l) >= max_jobs )); do
        sleep 0.5
    done

    (
        mkdir $this_id
        cd $this_id

        timeout 60s "$isobin" "$z"
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
