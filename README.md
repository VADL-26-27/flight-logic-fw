# flight-logic-fw

Flight Logic FW on STM32F413

System Clock: External Oscillator @ 25 MHz

## Programming the MCU

### To Clean the Build

```
make clean
```

### To Build

```
./build.sh
```

### To Flash

```
./build.sh --flash
```

## SPI Flash

Use USART2 with STLINK to communicate with Board

### To Read from SPI Flash

```
 python flight_log.py download <COM PORT> <Binary File Output Name>
```

Example:

```
 python flight_log.py download COM14 flight.bin
```

### To Parse Binary into CSV

```
 python flight_log.py download <Binary File Output Name> <CSV OUTPUT FILE NAME>
```

Example:

```
python flight_log.py parse flight.bin flight.csv
```

### To Plot a CSV

Requires `matplotlib` and `pandas`. Writes `flight.png` next to the CSV, replacing any existing file, unless you pass an output name.

```
python plot_flight.py flight.csv [output.png] [--title "My title"]
```

### To Erase SPI Flash

```
LOG CLEAR\n
```

Then send

```
LOG STATUS\n
```

Until the MCU returns

```
LOG READY\n
```
