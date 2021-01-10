## Before using this code
Do not forget to turn on monitor mode and choose the right channel on your wireless interface card.

Here is a exmaple on how to do it : 
```
sudo ifconfig wlp0s20f3 down
sudo iwconfig wlp0s20f3 mode monitor
sudo ifconfig wlp0s20f3 up
sudo iwconfig wlp0s20f3 channel 1 
```

## How to run the code
To compile the code just use :
```
make
```

And to run the program :
```
sudo ./bin/exec <wifi_interface> <output_file.csv>
```
If no name for the output file is specified, a file named `data.csv` will be generated in the base directory. The structure of the output file is the following:
```
esp_id , pc_timestamp , esp_timestamp , position , velocity
```

## Plot the Data
You can plot the resulting data using `matplot_data.py`. Example:
```
python3 matplot_data.py <csv_file>
```
You can also plot the receiving data in "real time" using `matplot_realtime.py`.
```
python3 matplot_realtime.py <csv_file>
```
In both cases you require the `matplotlib` and `pandas` libraries for Python.
