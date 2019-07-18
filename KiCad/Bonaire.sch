EESchema Schematic File Version 4
LIBS:Bonaire-cache
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 9
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 700  6700 1650 1150
U 5B10AF12
F0 "power" 60
F1 "power.sch" 60
$EndSheet
$Sheet
S 1150 1200 900  650 
U 5B2171DF
F0 "GPIOA" 60
F1 "GPIOA.sch" 60
F2 "RX485_A(-)" I R 2050 1600 60 
F3 "RX485_B(+)" I R 2050 1750 60 
$EndSheet
$Sheet
S 1150 2150 900  700 
U 5B2179B0
F0 "GPIOB" 60
F1 "GPIOB.sch" 60
$EndSheet
$Sheet
S 1150 3150 950  650 
U 5B21C79A
F0 "GPIOC" 60
F1 "GPIOC.sch" 60
$EndSheet
$Sheet
S 1150 4150 950  550 
U 5B21C79D
F0 "GPIOD" 60
F1 "GPIOD.sch" 60
$EndSheet
$Sheet
S 1200 5000 950  650 
U 5B21D278
F0 "GPIOE" 60
F1 "GPIOE.sch" 60
$EndSheet
$Sheet
S 1250 5900 950  550 
U 5B21D27B
F0 "GPIOF" 60
F1 "GPIOF.sch" 60
$EndSheet
Text Notes 8150 650  0    60   ~ 0
TOP
Text Label 5850 800  0    60   ~ 0
RS485A
Text Label 5700 950  0    60   ~ 0
RS485B
Wire Wire Line
	2050 1600 2600 1600
Wire Wire Line
	2600 1600 2600 800 
Wire Wire Line
	2600 800  7850 800 
Wire Wire Line
	7850 800  7850 1250
Wire Wire Line
	2050 1750 2800 1750
Wire Wire Line
	2800 1750 2800 950 
Wire Wire Line
	2800 950  7700 950 
Wire Wire Line
	7700 950  7700 1350
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5C8291D1
P 8800 1250
F 0 "J1" H 8880 1242 50  0000 L CNN
F 1 "Conn_01x04" H 8880 1151 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 8800 1250 50  0001 C CNN
F 3 "~" H 8800 1250 50  0001 C CNN
	1    8800 1250
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR01
U 1 1 5D1ADAB3
P 8400 1050
F 0 "#PWR01" H 8400 900 50  0001 C CNN
F 1 "+24V" H 8415 1223 50  0000 C CNN
F 2 "" H 8400 1050 50  0001 C CNN
F 3 "" H 8400 1050 50  0001 C CNN
	1    8400 1050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5D1ADB17
P 8400 1750
F 0 "#PWR02" H 8400 1500 50  0001 C CNN
F 1 "GND" H 8405 1577 50  0000 C CNN
F 2 "" H 8400 1750 50  0001 C CNN
F 3 "" H 8400 1750 50  0001 C CNN
	1    8400 1750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 1450 8400 1450
Wire Wire Line
	8400 1450 8400 1750
Wire Wire Line
	8600 1150 8400 1150
Wire Wire Line
	8400 1150 8400 1050
Wire Wire Line
	7850 1250 8600 1250
Wire Wire Line
	7700 1350 8600 1350
Text Notes 9050 1450 0    60   ~ 0
WHITE\nBLUE\nBROWN\nBLACK
$Comp
L Mechanical:MountingHole H2
U 1 1 5D1BC02C
P 6900 2900
F 0 "H2" H 7000 2946 50  0000 L CNN
F 1 "MountingHole" H 7000 2855 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 6900 2900 50  0001 C CNN
F 3 "~" H 6900 2900 50  0001 C CNN
	1    6900 2900
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5D1BC084
P 6900 3350
F 0 "H3" H 7000 3396 50  0000 L CNN
F 1 "MountingHole" H 7000 3305 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 6900 3350 50  0001 C CNN
F 3 "~" H 6900 3350 50  0001 C CNN
	1    6900 3350
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5D1BC0E1
P 6900 3850
F 0 "H4" H 7000 3896 50  0000 L CNN
F 1 "MountingHole" H 7000 3805 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 6900 3850 50  0001 C CNN
F 3 "~" H 6900 3850 50  0001 C CNN
	1    6900 3850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H1
U 1 1 5D1BC161
P 6850 4300
F 0 "H1" H 6950 4346 50  0000 L CNN
F 1 "MountingHole" H 6950 4255 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5" H 6850 4300 50  0001 C CNN
F 3 "~" H 6850 4300 50  0001 C CNN
	1    6850 4300
	1    0    0    -1  
$EndComp
Text Notes 4950 1250 0    60   ~ 0
NB!! - The internal RS485 chip has A and B mislabled!!\nto all the rest of our equipmnet they are turned around-\n
$EndSCHEMATC
