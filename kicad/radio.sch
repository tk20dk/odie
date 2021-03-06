EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 3 3
Title "Odie"
Date "2018-09-15"
Rev "v1.0"
Comp "www.2-0.dk"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 6250 2950 2    50   Output ~ 0
spi_miso
Text GLabel 6250 4250 2    50   Output ~ 0
spi_miso
Text GLabel 6250 3050 2    50   Input ~ 0
spi_mosi
Text GLabel 6250 4350 2    50   Input ~ 0
spi_mosi
Text GLabel 6250 2850 2    50   Input ~ 0
spi_sck
Text GLabel 6250 4150 2    50   Input ~ 0
spi_sck
Text GLabel 6250 3150 2    50   Input ~ 0
cs_radio433
Text GLabel 6250 4450 2    50   Input ~ 0
cs_radio868
Text GLabel 6250 3350 2    50   Output ~ 0
dio0_radio433
Text GLabel 6250 4650 2    50   Output ~ 0
dio0_radio868
$Comp
L power:+3V3 #PWR021
U 1 1 5B49272C
P 5350 2850
F 0 "#PWR021" H 5350 2700 50  0001 C CNN
F 1 "+3V3" V 5365 2978 50  0000 L CNN
F 2 "" H 5350 2850 50  0001 C CNN
F 3 "" H 5350 2850 50  0001 C CNN
	1    5350 2850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 5B49276D
P 5350 2950
F 0 "#PWR022" H 5350 2700 50  0001 C CNN
F 1 "GND" V 5355 2822 50  0000 R CNN
F 2 "" H 5350 2950 50  0001 C CNN
F 3 "" H 5350 2950 50  0001 C CNN
	1    5350 2950
	0    1    1    0   
$EndComp
$Comp
L power:+3V3 #PWR024
U 1 1 5B492929
P 5350 4150
F 0 "#PWR024" H 5350 4000 50  0001 C CNN
F 1 "+3V3" V 5365 4278 50  0000 L CNN
F 2 "" H 5350 4150 50  0001 C CNN
F 3 "" H 5350 4150 50  0001 C CNN
	1    5350 4150
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR025
U 1 1 5B49296A
P 5350 4250
F 0 "#PWR025" H 5350 4000 50  0001 C CNN
F 1 "GND" V 5355 4122 50  0000 R CNN
F 2 "" H 5350 4250 50  0001 C CNN
F 3 "" H 5350 4250 50  0001 C CNN
	1    5350 4250
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 3450 5350 3450
Wire Wire Line
	5350 3550 4950 3550
Wire Wire Line
	5350 4750 4950 4750
Wire Wire Line
	5350 4850 4950 4850
$Comp
L power:GND #PWR023
U 1 1 5B6589E4
P 5350 3050
F 0 "#PWR023" H 5350 2800 50  0001 C CNN
F 1 "GND" V 5355 2922 50  0000 R CNN
F 2 "" H 5350 3050 50  0001 C CNN
F 3 "" H 5350 3050 50  0001 C CNN
	1    5350 3050
	0    1    1    0   
$EndComp
Text GLabel 6250 3450 2    50   Output ~ 0
dio1_radio433
Text GLabel 6250 4750 2    50   Output ~ 0
dio1_radio868
Text GLabel 6250 4550 2    50   Input ~ 0
reset_radio868
Text GLabel 6250 3250 2    50   Input ~ 0
reset_radio433
Text GLabel 6250 3550 2    50   Output ~ 0
dio2_radio433
Text GLabel 6250 4850 2    50   Output ~ 0
dio2_radio868
$Comp
L power:GND #PWR?
U 1 1 5B66F661
P 3500 3900
AR Path="/5B48B53E/5B66F661" Ref="#PWR?"  Part="1" 
AR Path="/5B48B545/5B66F661" Ref="#PWR018"  Part="1" 
F 0 "#PWR018" H 3500 3650 50  0001 C CNN
F 1 "GND" V 3505 3772 50  0000 R CNN
F 2 "" H 3500 3900 50  0001 C CNN
F 3 "" H 3500 3900 50  0001 C CNN
	1    3500 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5B66F667
P 3000 3900
AR Path="/5B48B53E/5B66F667" Ref="#PWR?"  Part="1" 
AR Path="/5B48B545/5B66F667" Ref="#PWR017"  Part="1" 
F 0 "#PWR017" H 3000 3650 50  0001 C CNN
F 1 "GND" V 3005 3772 50  0000 R CNN
F 2 "" H 3000 3900 50  0001 C CNN
F 3 "" H 3000 3900 50  0001 C CNN
	1    3000 3900
	0    1    1    0   
$EndComp
Text GLabel 3500 4100 2    50   Output ~ 0
swclk
Text GLabel 3500 4000 2    50   BiDi ~ 0
swdio
Text GLabel 3000 4100 0    50   Output ~ 0
uart_rx
Text GLabel 3000 4000 0    50   Input ~ 0
uart_tx
$Comp
L Device:C C8
U 1 1 5B7B200D
P 3000 2450
F 0 "C8" H 3115 2496 50  0000 L CNN
F 1 "1u" H 3115 2405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3038 2300 50  0001 C CNN
F 3 "~" H 3000 2450 50  0001 C CNN
	1    3000 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 5B7B2079
P 3400 2450
F 0 "C9" H 3515 2496 50  0000 L CNN
F 1 "1u" H 3515 2405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3438 2300 50  0001 C CNN
F 3 "~" H 3400 2450 50  0001 C CNN
	1    3400 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5B7B2137
P 3000 2600
F 0 "#PWR020" H 3000 2350 50  0001 C CNN
F 1 "GND" H 3005 2427 50  0000 C CNN
F 2 "" H 3000 2600 50  0001 C CNN
F 3 "" H 3000 2600 50  0001 C CNN
	1    3000 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5B7B2157
P 3400 2600
F 0 "#PWR027" H 3400 2350 50  0001 C CNN
F 1 "GND" H 3405 2427 50  0000 C CNN
F 2 "" H 3400 2600 50  0001 C CNN
F 3 "" H 3400 2600 50  0001 C CNN
	1    3400 2600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR028
U 1 1 5B7B2170
P 4750 3650
F 0 "#PWR028" H 4750 3400 50  0001 C CNN
F 1 "GND" H 4755 3477 50  0000 C CNN
F 2 "" H 4750 3650 50  0001 C CNN
F 3 "" H 4750 3650 50  0001 C CNN
	1    4750 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 5B7B21F6
P 4750 4950
F 0 "#PWR029" H 4750 4700 50  0001 C CNN
F 1 "GND" H 4755 4777 50  0000 C CNN
F 2 "" H 4750 4950 50  0001 C CNN
F 3 "" H 4750 4950 50  0001 C CNN
	1    4750 4950
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR026
U 1 1 5B7B2311
P 3400 2300
F 0 "#PWR026" H 3400 2150 50  0001 C CNN
F 1 "+3.3V" H 3415 2473 50  0000 C CNN
F 2 "" H 3400 2300 50  0001 C CNN
F 3 "" H 3400 2300 50  0001 C CNN
	1    3400 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR019
U 1 1 5B7B2331
P 3000 2300
F 0 "#PWR019" H 3000 2150 50  0001 C CNN
F 1 "+3.3V" H 3015 2473 50  0000 C CNN
F 2 "" H 3000 2300 50  0001 C CNN
F 3 "" H 3000 2300 50  0001 C CNN
	1    3000 2300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x04_Odd_Even J2
U 1 1 5B9D1226
P 3200 3900
F 0 "J2" H 3250 4217 50  0000 C CNN
F 1 "Debug" H 3250 4126 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical" H 3200 3900 50  0001 C CNN
F 3 "~" H 3200 3900 50  0001 C CNN
	1    3200 3900
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR030
U 1 1 5B9D12AB
P 3000 3800
F 0 "#PWR030" H 3000 3650 50  0001 C CNN
F 1 "+3V3" V 3015 3928 50  0000 L CNN
F 2 "" H 3000 3800 50  0001 C CNN
F 3 "" H 3000 3800 50  0001 C CNN
	1    3000 3800
	0    -1   -1   0   
$EndComp
$Comp
L power:+3V3 #PWR031
U 1 1 5B9D12C4
P 3500 3800
F 0 "#PWR031" H 3500 3650 50  0001 C CNN
F 1 "+3V3" V 3515 3928 50  0000 L CNN
F 2 "" H 3500 3800 50  0001 C CNN
F 3 "" H 3500 3800 50  0001 C CNN
	1    3500 3800
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_Coaxial ANT1
U 1 1 5FAE931B
P 4750 3450
F 0 "ANT1" H 4678 3688 50  0000 C CNN
F 1 "SMA 433MHz" H 4678 3597 50  0000 C CNN
F 2 "odie:SMA_Molex_73251-2200_Horizontal" H 4750 3450 50  0001 C CNN
F 3 " ~" H 4750 3450 50  0001 C CNN
	1    4750 3450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 3550 4950 3650
Wire Wire Line
	4950 3650 4750 3650
Connection ~ 4750 3650
$Comp
L Connector:Conn_Coaxial ANT2
U 1 1 5FAF0A6E
P 4750 4750
F 0 "ANT2" H 4678 4988 50  0000 C CNN
F 1 "SMA 868MHz" H 4678 4897 50  0000 C CNN
F 2 "odie:SMA_Molex_73251-2200_Horizontal" H 4750 4750 50  0001 C CNN
F 3 " ~" H 4750 4750 50  0001 C CNN
	1    4750 4750
	-1   0    0    -1  
$EndComp
Wire Wire Line
	4950 4850 4950 4950
Wire Wire Line
	4950 4950 4750 4950
Connection ~ 4750 4950
$Comp
L odie:LoRa1276-433 U4
U 1 1 5FAA67C3
P 5800 3200
F 0 "U4" H 5800 3815 50  0000 C CNN
F 1 "LoRa1276-433" H 5800 3724 50  0000 C CNN
F 2 "odie:NicerRF-LoRa1276" H 5800 3200 50  0001 C CNN
F 3 "" H 5800 3200 50  0001 C CNN
	1    5800 3200
	1    0    0    -1  
$EndComp
$Comp
L odie:LoRa1278-868 U5
U 1 1 5FAAA4A8
P 5800 4500
F 0 "U5" H 5800 5115 50  0000 C CNN
F 1 "LoRa1278-868" H 5800 5024 50  0000 C CNN
F 2 "odie:NicerRF-LoRa1276" H 5800 4500 50  0001 C CNN
F 3 "" H 5800 4500 50  0001 C CNN
	1    5800 4500
	1    0    0    -1  
$EndComp
$EndSCHEMATC
