EESchema Schematic File Version 4
LIBS:Bonaire-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 5 9
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
L stm32:STM32F303V(B-C)Tx_u U2
U 2 1 5B217BB3
P 7050 5350
F 0 "U2" H 7800 4250 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 7050 4400 50  0000 C CNN
F 2 "Housings_QFP:LQFP-100_14x14mm_Pitch0.5mm" H 7050 4200 50  0000 C CIN
F 3 "" H 7050 5350 50  0000 C CNN
	2    7050 5350
	-1   0    0    1   
$EndComp
$Comp
L power1:GND #PWR023
U 1 1 5B217BC1
P 1350 5850
F 0 "#PWR023" H 1350 5600 50  0001 C CNN
F 1 "GND" H 1350 5700 50  0000 C CNN
F 2 "" H 1350 5850 50  0001 C CNN
F 3 "" H 1350 5850 50  0001 C CNN
	1    1350 5850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 4650 3750 4650
Wire Wire Line
	1500 4850 3750 4850
Wire Wire Line
	1900 5050 3750 5050
Wire Wire Line
	1800 4750 3750 4750
Text Label 2500 4750 0    60   ~ 12
MISO
Text Label 2100 4650 0    60   ~ 12
MOSI
Text Label 2100 4850 0    60   ~ 12
SCK
Text Label 2100 5050 0    60   ~ 12
~CS
Text Label 3100 5450 0    60   ~ 0
UART1_RX
Text Label 3300 5550 0    60   ~ 0
UART1_TX
Wire Wire Line
	1200 5850 1350 5850
NoConn ~ 1200 5350
NoConn ~ 1200 5650
NoConn ~ 1200 5750
Wire Wire Line
	3750 5550 1750 5550
Wire Wire Line
	1750 5550 1750 5450
Wire Wire Line
	1750 5450 1200 5450
Wire Wire Line
	3750 5450 2050 5450
Wire Wire Line
	2050 5450 2050 5650
Wire Wire Line
	2050 5650 1550 5650
Wire Wire Line
	1550 5650 1550 5550
Wire Wire Line
	1550 5550 1200 5550
$Comp
L Connector_Generic:Conn_01x06 J8
U 1 1 5C826A70
P 1000 5650
F 0 "J8" H 920 5125 50  0000 C CNN
F 1 "Conn_01x06" H 920 5216 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical" H 1000 5650 50  0001 C CNN
F 3 "~" H 1000 5650 50  0001 C CNN
	1    1000 5650
	-1   0    0    1   
$EndComp
NoConn ~ 3750 4950
NoConn ~ 3750 5150
NoConn ~ 3750 5250
NoConn ~ 3750 5350
NoConn ~ 3750 5650
NoConn ~ 3750 5750
NoConn ~ 3750 5850
NoConn ~ 3750 5950
NoConn ~ 3750 6050
NoConn ~ 3750 6150
$EndSCHEMATC
