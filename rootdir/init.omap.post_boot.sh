#!/system/bin/sh
echo "lazy" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo 1 > /sys/devices/system/cpu/cpu0/cpufreq/lazy/screenoff_maxfreq
#echo 85 > /sys/devices/system/cpu/cpu0/cpufreq/ondemand/up_threshold
#echo 30 > /sys/devices/system/cpu/cpu0/cpufreq/ondemand/down_differential
#echo 50000 > /sys/devices/system/cpu/cpu0/cpufreq/ondemand/sampling_rate
