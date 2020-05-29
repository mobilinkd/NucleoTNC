EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
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
L Mechanical:MountingHole_Pad H1
U 1 1 5E70BE26
P 3450 1850
F 0 "H1" H 3550 1899 50  0000 L CNN
F 1 "MountingHole_Pad" H 3550 1808 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 3450 1850 50  0001 C CNN
F 3 "~" H 3450 1850 50  0001 C CNN
	1    3450 1850
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H2
U 1 1 5E70BFA6
P 3450 2200
F 0 "H2" H 3550 2249 50  0000 L CNN
F 1 "MountingHole_Pad" H 3550 2158 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 3450 2200 50  0001 C CNN
F 3 "~" H 3450 2200 50  0001 C CNN
	1    3450 2200
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H3
U 1 1 5E70C460
P 3450 2550
F 0 "H3" H 3550 2599 50  0000 L CNN
F 1 "MountingHole_Pad" H 3550 2508 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 3450 2550 50  0001 C CNN
F 3 "~" H 3450 2550 50  0001 C CNN
	1    3450 2550
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad H4
U 1 1 5E70C65D
P 3450 2900
F 0 "H4" H 3550 2949 50  0000 L CNN
F 1 "MountingHole_Pad" H 3550 2858 50  0000 L CNN
F 2 "MountingHole:MountingHole_2.7mm_M2.5_ISO14580_Pad" H 3450 2900 50  0001 C CNN
F 3 "~" H 3450 2900 50  0001 C CNN
	1    3450 2900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5E70C921
P 3150 3000
F 0 "#PWR01" H 3150 2750 50  0001 C CNN
F 1 "GND" H 3155 2827 50  0000 C CNN
F 2 "" H 3150 3000 50  0001 C CNN
F 3 "" H 3150 3000 50  0001 C CNN
	1    3150 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 3000 3150 3000
Wire Wire Line
	3450 2650 3150 2650
Wire Wire Line
	3150 2650 3150 3000
Connection ~ 3150 3000
Wire Wire Line
	3450 2300 3150 2300
Wire Wire Line
	3150 2300 3150 2650
Connection ~ 3150 2650
Wire Wire Line
	3450 1950 3150 1950
Wire Wire Line
	3150 1950 3150 2300
Connection ~ 3150 2300
$Comp
L Mechanical:MountingHole H5
U 1 1 5E70D423
P 4900 1800
F 0 "H5" H 5000 1846 50  0000 L CNN
F 1 "Radio" H 5000 1755 50  0000 L CNN
F 2 "MountingHole:MountingHole_6mm" H 4900 1800 50  0001 C CNN
F 3 "~" H 4900 1800 50  0001 C CNN
	1    4900 1800
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H6
U 1 1 5E70D94E
P 4900 2150
F 0 "H6" H 5000 2196 50  0000 L CNN
F 1 "Reset" H 5000 2105 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4" H 4900 2150 50  0001 C CNN
F 3 "~" H 4900 2150 50  0001 C CNN
	1    4900 2150
	1    0    0    -1  
$EndComp
$EndSCHEMATC
