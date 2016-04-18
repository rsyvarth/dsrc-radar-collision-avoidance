#!/bin/bash

dirs=$(find $1 -mindepth 1 -type d)

for dir in $dirs; do
	if [[ $dir == *"export"* ]]
	then
		echo $dir
		$(python main.py --visualize $dir --export_interval 1000 --export_path $dir/frames)
	fi
done
