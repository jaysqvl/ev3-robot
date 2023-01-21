# Lego EV3 Robot Code Repository
GIF Example Demonstrations Coming...

## Linetrack Mode (Python or C)
<img src="https://github.com/jsqvl/EV3-Robot/blob/master/Demo-Media/Linetracker-demo-compressed.gif?raw=true" width="811" height="360" />

Tracks colored lines on white backdrops (Based on color reflectivity/brightness).
C or Python code available
The python version takes longer to initialize and is generally less accurate due to  slower color polling speed

## Barcode Scanning Mode (C/Matlab)
![Barcode Scanner Demo Gif](https://github.com/jsqvl/EV3-Robot/blob/master/Demo-Media/Maze-solver-demo.gif?raw=true)

When compiled, place the barcode scanning robot at either end of the barcode. The robot will collect the data automatically by running forward over it and store it. Returning to your computer, the collected data is run against a barcode dictionary where it will identify the scanned code.

## Maze-solving Mode (C)
![Maze-solver Demo GIF](https://github.com/jsqvl/EV3-Robot/blob/master/Demo-Media/barcodescanner-demo.gif?raw=true)

Uses LRRR Maze path discovery algorithm to traverse the entire maze. It subsequently calculates the shortest path to its initialized position in the maze and returns there. The robot movement alignment-correction algorithm uses its surrounding cell to align with the center of the cell so that it doesn't go astray.
