EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:U_depot-cache
EELAYER 25 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L D_Schottky D?
U 1 1 5883CF6D
P 2750 3700
F 0 "D?" H 2750 3800 50  0000 C CNN
F 1 "LSM115JE3/TR13" H 2750 3600 50  0000 C CNN
F 2 "" H 2750 3700 50  0000 C CNN
F 3 "" H 2750 3700 50  0000 C CNN
	1    2750 3700
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5883CFC8
P 3200 3950
F 0 "R?" V 3280 3950 50  0000 C CNN
F 1 "R" V 3200 3950 50  0000 C CNN
F 2 "" V 3130 3950 50  0000 C CNN
F 3 "" H 3200 3950 50  0000 C CNN
	1    3200 3950
	1    0    0    -1  
$EndComp
$Comp
L ZENER D?
U 1 1 5883CFE9
P 3200 4450
F 0 "D?" H 3200 4550 50  0000 C CNN
F 1 "ZENER" H 3200 4350 50  0000 C CNN
F 2 "" H 3200 4450 50  0000 C CNN
F 3 "" H 3200 4450 50  0000 C CNN
	1    3200 4450
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5883D03A
P 3200 4700
F 0 "#PWR?" H 3200 4450 50  0001 C CNN
F 1 "GND" H 3200 4550 50  0000 C CNN
F 2 "" H 3200 4700 50  0000 C CNN
F 3 "" H 3200 4700 50  0000 C CNN
	1    3200 4700
	1    0    0    -1  
$EndComp
$Comp
L OPA376AIDCK U?
U 1 1 5883D06C
P 4850 4000
F 0 "U?" H 4900 4200 50  0000 C CNN
F 1 "LMC7101BYM5-TR" H 5050 3800 50  0000 C CNN
F 2 "SC70-5" H 4800 3700 50  0000 L CNN
F 3 "" H 4900 4200 50  0000 C CNN
F 4 "LMC7101BYM5-TR" H 4850 4000 60  0001 C CNN "Part Number"
	1    4850 4000
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BCE Q?
U 1 1 5883D0DB
P 5800 4000
F 0 "Q?" H 6100 4050 50  0000 R CNN
F 1 "Q_NPN_BCE" H 6400 3950 50  0000 R CNN
F 2 "" H 6000 4100 50  0000 C CNN
F 3 "" H 5800 4000 50  0000 C CNN
	1    5800 4000
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5883D158
P 6050 4500
F 0 "C?" H 6075 4600 50  0000 L CNN
F 1 "C" H 6075 4400 50  0000 L CNN
F 2 "" H 6088 4350 50  0000 C CNN
F 3 "" H 6050 4500 50  0000 C CNN
	1    6050 4500
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D?
U 1 1 5883D1D1
P 6650 3950
F 0 "D?" H 6650 4050 50  0000 C CNN
F 1 "LSM115JE3/TR13" H 6650 3850 50  0000 C CNN
F 2 "" H 6650 3950 50  0000 C CNN
F 3 "" H 6650 3950 50  0000 C CNN
F 4 "LSM115JE3/TR13" H 6650 3950 60  0001 C CNN "Part Number"
	1    6650 3950
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5883D248
P 6050 4700
F 0 "#PWR?" H 6050 4450 50  0001 C CNN
F 1 "GND" H 6050 4550 50  0000 C CNN
F 2 "" H 6050 4700 50  0000 C CNN
F 3 "" H 6050 4700 50  0000 C CNN
	1    6050 4700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5883D63D
P 4750 4350
F 0 "#PWR?" H 4750 4100 50  0001 C CNN
F 1 "GND" H 4750 4200 50  0000 C CNN
F 2 "" H 4750 4350 50  0000 C CNN
F 3 "" H 4750 4350 50  0000 C CNN
	1    4750 4350
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5883D288
P 2950 3950
F 0 "R?" V 3030 3950 50  0000 C CNN
F 1 "82k" V 2950 3950 50  0000 C CNN
F 2 "" V 2880 3950 50  0000 C CNN
F 3 "" H 2950 3950 50  0000 C CNN
	1    2950 3950
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5883D2BD
P 2950 4450
F 0 "R?" V 3030 4450 50  0000 C CNN
F 1 "68k" V 2950 4450 50  0000 C CNN
F 2 "" V 2880 4450 50  0000 C CNN
F 3 "" H 2950 4450 50  0000 C CNN
	1    2950 4450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5883D337
P 2950 4700
F 0 "#PWR?" H 2950 4450 50  0001 C CNN
F 1 "GND" H 2950 4550 50  0000 C CNN
F 2 "" H 2950 4700 50  0000 C CNN
F 3 "" H 2950 4700 50  0000 C CNN
	1    2950 4700
	1    0    0    -1  
$EndComp
Text GLabel 2900 4200 0    60   Input ~ 0
ANALOG_READ
$Comp
L R R?
U 1 1 589F77FF
P 5550 4150
F 0 "R?" V 5630 4150 50  0000 C CNN
F 1 "R" V 5550 4150 50  0000 C CNN
F 2 "" V 5480 4150 50  0000 C CNN
F 3 "" H 5550 4150 50  0000 C CNN
	1    5550 4150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 589F7930
P 5550 4300
F 0 "#PWR?" H 5550 4050 50  0001 C CNN
F 1 "GND" H 5550 4150 50  0000 C CNN
F 2 "" H 5550 4300 50  0000 C CNN
F 3 "" H 5550 4300 50  0000 C CNN
	1    5550 4300
	1    0    0    -1  
$EndComp
$Comp
L Q_NMOS_DGS Q?
U 1 1 589F7A2A
P 6500 4550
F 0 "Q?" H 6700 4600 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 6700 4500 50  0000 L CNN
F 2 "" H 6700 4650 50  0000 C CNN
F 3 "" H 6500 4550 50  0000 C CNN
	1    6500 4550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P?
U 1 1 589F7AA7
P 6800 5350
F 0 "P?" H 6800 5500 50  0000 C CNN
F 1 "CONN_01X02" V 6900 5350 50  0000 C CNN
F 2 "" H 6800 5350 50  0000 C CNN
F 3 "" H 6800 5350 50  0000 C CNN
	1    6800 5350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 589F7ED7
P 6600 5400
F 0 "#PWR?" H 6600 5150 50  0001 C CNN
F 1 "GND" H 6600 5250 50  0000 C CNN
F 2 "" H 6600 5400 50  0000 C CNN
F 3 "" H 6600 5400 50  0000 C CNN
	1    6600 5400
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D?
U 1 1 589F8A13
P 6850 3700
F 0 "D?" H 6850 3800 50  0000 C CNN
F 1 "D_Schottky" H 6850 3600 50  0000 C CNN
F 2 "" H 6850 3700 50  0000 C CNN
F 3 "" H 6850 3700 50  0000 C CNN
	1    6850 3700
	-1   0    0    1   
$EndComp
$Comp
L C C?
U 1 1 589F8BEA
P 7150 3850
F 0 "C?" H 7175 3950 50  0000 L CNN
F 1 "C" H 7175 3750 50  0000 L CNN
F 2 "" H 7188 3700 50  0000 C CNN
F 3 "" H 7150 3850 50  0000 C CNN
	1    7150 3850
	1    0    0    -1  
$EndComp
$Comp
L D_Schottky D?
U 1 1 589F8CCD
P 7450 3700
F 0 "D?" H 7450 3800 50  0000 C CNN
F 1 "D_Schottky" H 7450 3600 50  0000 C CNN
F 2 "" H 7450 3700 50  0000 C CNN
F 3 "" H 7450 3700 50  0000 C CNN
	1    7450 3700
	-1   0    0    1   
$EndComp
$Comp
L C C?
U 1 1 589F8CD3
P 7750 3850
F 0 "C?" H 7775 3950 50  0000 L CNN
F 1 "C" H 7775 3750 50  0000 L CNN
F 2 "" H 7788 3700 50  0000 C CNN
F 3 "" H 7750 3850 50  0000 C CNN
	1    7750 3850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 589F8DF2
P 7750 4000
F 0 "#PWR?" H 7750 3750 50  0001 C CNN
F 1 "GND" H 7750 3850 50  0000 C CNN
F 2 "" H 7750 4000 50  0000 C CNN
F 3 "" H 7750 4000 50  0000 C CNN
	1    7750 4000
	1    0    0    -1  
$EndComp
Text GLabel 7150 4150 3    60   Input ~ 0
Teensy_charge_pump
$Comp
L Q_NMOS_DGS Q?
U 1 1 589F9AAC
P 8600 3900
F 0 "Q?" H 8800 3950 50  0000 L CNN
F 1 "Q_NMOS_DGS" H 8800 3850 50  0000 L CNN
F 2 "" H 8800 4000 50  0000 C CNN
F 3 "" H 8600 3900 50  0000 C CNN
	1    8600 3900
	0    -1   1    0   
$EndComp
$Comp
L R R?
U 1 1 589F9B70
P 8850 3850
F 0 "R?" V 8930 3850 50  0000 C CNN
F 1 "3.3k" V 8850 3850 50  0000 C CNN
F 2 "" V 8780 3850 50  0000 C CNN
F 3 "" H 8850 3850 50  0000 C CNN
	1    8850 3850
	-1   0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 589F9C11
P 8350 3850
F 0 "R?" V 8430 3850 50  0000 C CNN
F 1 "100k" V 8350 3850 50  0000 C CNN
F 2 "" V 8280 3850 50  0000 C CNN
F 3 "" H 8350 3850 50  0000 C CNN
	1    8350 3850
	-1   0    0    -1  
$EndComp
Text Label 9000 3700 2    60   ~ 0
3.3V
Text Label 9000 4000 0    60   ~ 0
Teensy_GPIO
Wire Wire Line
	5900 4200 5900 4350
Wire Wire Line
	5900 4350 6650 4350
Wire Wire Line
	6650 4350 6650 4100
Connection ~ 6050 4350
Wire Wire Line
	6050 4700 6050 4650
Wire Wire Line
	2900 3700 6700 3700
Wire Wire Line
	5900 3700 5900 3800
Wire Wire Line
	6650 3700 6650 3800
Connection ~ 5900 3700
Wire Wire Line
	5150 4000 5600 4000
Wire Wire Line
	4550 4100 4350 4100
Wire Wire Line
	4350 4100 4350 3700
Connection ~ 4750 3700
Wire Wire Line
	3200 3700 3200 3800
Connection ~ 4350 3700
Connection ~ 3200 3700
Wire Wire Line
	3200 4100 3200 4250
Wire Wire Line
	3200 4700 3200 4650
Wire Wire Line
	4550 3900 3450 3900
Wire Wire Line
	3450 3900 3450 4200
Wire Wire Line
	3450 4200 3200 4200
Connection ~ 3200 4200
Wire Wire Line
	4750 4350 4750 4300
Wire Wire Line
	2950 4100 2950 4300
Wire Wire Line
	2950 4700 2950 4600
Wire Wire Line
	2950 3800 2950 3700
Connection ~ 2950 3700
Wire Wire Line
	2900 4200 2950 4200
Connection ~ 2950 4200
Connection ~ 5550 4000
Wire Wire Line
	6600 4750 6600 5300
Connection ~ 6650 3700
Wire Wire Line
	7000 3700 7300 3700
Wire Wire Line
	7600 3700 8350 3700
Wire Wire Line
	7150 4000 7150 4150
Wire Wire Line
	8800 4000 9000 4000
Wire Wire Line
	8150 4000 8400 4000
Connection ~ 8350 4000
Connection ~ 8850 4000
Wire Wire Line
	8600 3700 9000 3700
Wire Wire Line
	8150 4000 8150 5650
Wire Wire Line
	8150 5650 6300 5650
Wire Wire Line
	6300 5650 6300 4550
Connection ~ 8850 3700
Connection ~ 6600 4350
Connection ~ 7750 3700
Connection ~ 7150 3700
$EndSCHEMATC
