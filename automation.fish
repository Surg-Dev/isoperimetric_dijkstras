#!/usr/bin/fish
rm -rf db-out
mkdir db-out
cd db-out
set filec 1
set isobin /home/srg/Projects/isoperimetric_dijkstras/build/bin/iso_dij
for file in /home/srg/Projects/databases/MeshsegBenchmark-1.0/MeshsegBenchmark-1.0/data/off/*
    # echo $file
    mkdir model$filec
    cd model$filec
    timeout 10s $isobin $file
    cd ..
    set filec (math $filec+1)
end
