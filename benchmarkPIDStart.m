%% SOC PID benchmark configuration file
%initial condition PI control
kpo=0.2;
kio=0.6;
% PI controller gains limits
maxKp=0.1; minKp=0.01;
maxKi=0.1; minKi=0.001;
%Globalized Constrained Nelder Mead GCNM parameters
GCNMEnable=1;
optimPeriod=300;        %optimization execution period (s)
resetThreshold=0.1;     %probabilistic restart threshold
refSignalPeriod=300;    %Reference signal period
%constraints limits
OVLim=0.05;             %5% overshoot
TSLim=70;               %Settling time limit (s)
L=0.1;                  %delay value L=0.1,1,10;
N=100;                  %derivative filter N (Zero for PI controller)


%% Run the Benchmark file
sim("SOCPIControlBenchmark.slx")

% SOC PI time response and performance indices calculation
benchmarkFiguresSOC


