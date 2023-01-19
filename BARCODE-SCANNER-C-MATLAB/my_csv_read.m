%%
clc;
clear;
close all;
array = csvread('B.csv');
colorArray = array(:,4);
plot(colorArray);
hold on

%Assume window size of 20
ws = 20;

%THIS SMOOTHENS THE GRAPH
% Calculating average of each window (bar), looping through the entire CSV file
% It creates a new array of bars that numerically represent their average values
for i = 1: length(colorArray)-ws
    colorArrayAvg(i)= sum(colorArray(i:i+ws))/ws;
    %fprintf('%f \n', colorArrayAvg(i));
end
%plot(colorArrayAvg);
%hold on

%THIS FINDS CHANGE IN THE GRAPH, WILL RESULT IN PEAKS
% Calculating derivative of each previously calculated bars average
for i = 1:length(colorArrayAvg)-1
    colorDerivative(i) = abs(colorArrayAvg(i+1) - colorArrayAvg(i));
end

% Plots the color derivative
%plot(colorDerivative);
%hold on

%THIS SMOOTHENS THE PEAKS by averaging the derivative over the window size again for each interval
%Smoothening derivative out, similar to what we did for colorArrayAvg
for i = 1: length(colorDerivative)-ws
    colorDerAvg(i)= sum(colorDerivative(i:i+ws))/ws;
    %fprintf('%f \n', colorArrayAvg(i));
end
plot(colorDerAvg);
hold off

%THIS STORES THE PEAKS AND THE RESPECTIVE INDICES
% Find the peaks of the derivative, minimum distance between peaks = 10,
% minimum height for peak = 1
[pks,locs] = findpeaks(colorDerAvg, 'MinPeakDistance',10, 'MinPeakHeight', 1);

g=sprintf("%d ", locs);
fprintf("%s\n", g);

% starting point of each section in the matrix that represents a number, gets updated in each loop
start = 0;
size = length(colorArray); % total length of matrix or number of rows in CSV
binary_str = ""; % string that will store the binary representation of data
index = 1;

%NOW HAVE TO DETERMINE THE WIDTH
% Loop through the array of peaks location using index

while (index <= length(locs))
 % Odd indices to calculate black regions easily
    if(mod(index,2) ~= 0)
        black_region = locs(index) - start;
        
        % Calculate total region
        % if next index is out of bound, use size of matrix to calculate
        % total region. Otherwise calculate using next or even index.
        if(index + 1 > length(locs))
            total_region = size - start;
        else
            total_region = locs(index + 1) - start;
        end
        
        % ratio either 2:3 ('1') or 1:3 ('0')
        ratio = black_region / total_region; 
        fprintf("%f\n",ratio);
        if(ratio < 0.55)
            binary_str = append(binary_str, "0");
        else
            binary_str = append(binary_str, "1");
        end
    else
        % Even indices are starting point for each region
        start = locs(index);
    end
    index = index + 1;    
end
fprintf(binary_str);