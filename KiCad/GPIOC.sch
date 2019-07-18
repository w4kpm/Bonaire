EESchema Schematic File Version 4
LIBS:Bonaire-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 9
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
U 3 1 5B21CD88
P 8400 2000
F 0 "U2" H 9000 1050 50  0000 C CNN
F 1 "STM32F303V(B-C)Tx_u" H 8400 1100 50  0000 C CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 8450 1000 50  0000 C CIN
F 3 "" H 8400 2000 50  0000 C CNN
	3    8400 2000
	-1   0    0    1   
$EndComp
NoConn ~ 5900 1300
NoConn ~ 5900 1400
NoConn ~ 5900 1500
NoConn ~ 5900 2000
NoConn ~ 5900 2300
NoConn ~ 5900 2100
NoConn ~ 5900 2200
NoConn ~ 5900 2400
NoConn ~ 5900 2500
NoConn ~ 5900 2600
NoConn ~ 5900 2700
NoConn ~ 5900 2800
$Comp
L Connector_Generic:Conn_01x08 J4
U 1 1 5D1B4B7E
P 3850 1700
F 0 "J4" H 3770 1075 50  0000 C CNN
F 1 "Conn_01x08" H 3770 1166 50  0000 C CNN
F 2 "Connector_Molex:Molex_PicoBlade_53261-0871_1x08-1MP_P1.25mm_Horizontal" H 3850 1700 50  0001 C CNN
F 3 "~" H 3850 1700 50  0001 C CNN
	1    3850 1700
	-1   0    0    -1  
$EndComp
Text Notes 3350 2150 0    60   ~ 0
NC\nNC\nRST\nTXD\nRXD\nSET\nGND\nVCC
$Comp
L power:+5V #PWR023
U 1 1 5D1B4C53
P 4250 2100
F 0 "#PWR023" H 4250 1950 50  0001 C CNN
F 1 "+5V" V 4265 2228 50  0000 L CNN
F 2 "" H 4250 2100 50  0001 C CNN
F 3 "" H 4250 2100 50  0001 C CNN
	1    4250 2100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5D1B4CC4
P 4250 2000
F 0 "#PWR022" H 4250 1750 50  0001 C CNN
F 1 "GND" V 4255 1872 50  0000 R CNN
F 2 "" H 4250 2000 50  0001 C CNN
F 3 "" H 4250 2000 50  0001 C CNN
	1    4250 2000
	0    -1   -1   0   
$EndComp
NoConn ~ 4050 1400
NoConn ~ 4050 1500
Wire Wire Line
	4050 1700 5900 1700
Wire Wire Line
	4050 1800 5900 1800
Wire Wire Line
	4050 1600 5900 1600
Wire Wire Line
	5900 1900 4050 1900
Wire Wire Line
	4050 2000 4250 2000
Wire Wire Line
	4050 2100 4250 2100
$EndSCHEMATC
