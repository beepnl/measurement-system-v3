2020_02_26 Release van Versie 1.3.5
	De DS18B20 communicatie fout teller wordt nu gereset bij een nieuwe staat of conversie start. Voorheen gebeurde enkel na het succesvol uitlezen van de temperatuur. Dit had als gevolg dat als een DS18B20 
	meer dan 5 communicatie fouten had, dan stopte de sensor met meten bij oudere firmware.
	
	Bij de conversie start wordt nu de laatst gemeten temperatuur op -100.0 gezet, dus als de sensor niet uitgelezen kan worden zal die waarde worden verstuurd met LoRaWAN in plaats van de laatst gemeten waarde.

2020_01_28 Release van Versie 1.3.4
	READ_APPLICATION_CONFIG data toegevoegd aan het opstart bericht.
	
	Een minuut na het succesvol verbinden met het LoRaWAN netwerk wordt het opstart bericht verstuurd. Mocht dit niet lukken in verband met de duty cycle wordt dit 
		een minuut later opnieuw geprobeerd.

2020_01_28 Release van Versie 1.3.3
	Standaard bericht interval is nu 15 minuten in plaats van 1 minuut.
		Dit betekend wel dat reeds geprogrameerde sensoren die niet worden gewist en opnieuw worden geprogrameerd met de hexfile, bijvoorbeeld met DFU nog steeds de 
		standaard interval van 1 minuut hebben na het updaten naar 1.3.3.
	
	Opstart bericht met FPORT waarde 2 (BEEP_SENSOR_ON) wordt nu weer na het opstarten en joinen van het netwerk verstuurd.
	
	

2020_01_06 Release van Versie 1.3.2
	Fix voor de BME280 die niet correct werkte en daardoor continue hetzelfde resultaat terug gaf. 
	
	> Pim had een probleem met de tilt sensor gerapporteerd, die niet werkte als de BEEPBASE voor langere tijd stil lag. Dit heb ik zelf niet kunnen reproduceren met de print vanuit China.
		Ook niet over periodes van enkele dagen.
		
		

2019_12_23 Release van Versie 1.3.1
	Bericht interval gebruikt nu weer de normale vermenigvuldigingsfactor van 60 ipv de debug waarde van 20.

	BME280 statemachine verder uitgebreid met extra controles voor het geval dat er geen sensor is aangesloten.
	
	LoRaWAN dutycycle limitatie werkt nu en kan aan en uit gezet worden.
	
	LoRaWAN uitschakelen resette niet de LoRaWAN stack. Het wijzigen van de LoRaWAN configuratie met WRITE_LORAWAN_STATE reset nu altijd de LoRaWAN stack.
	
	De bericht ratio werkte niet naar behoren omdat de statemachine in dezelfde state bleef hangen en na een aantal iteraties toch een bericht verstuurde.
		Het eerste start-up bericht en downlink-response berichten negeren nu de bericht ratio en worden altijd verstuurd op het eerst volgende bericht interval.
		
	De tilt sensor op de nieuwe PCB's moet nu correct werken. De applicatie onthoud nu de laatste opstart staat. Als dit vertikaal was en de huidige orientatie is ook
		vertikaal dan is de BEEPBASE wakker geworden door een trilling en gaat de BEEPBASE direct naar SYSTEM OFF zonder buzzer geluid patroon.
	
	

2019_12_20 Release van Versie 1.3.0
	TLV Audio ADC toegevoegd met FFT functionaliteit.
		Alle drie de kanalen werken, zowel Links als rechts.
		Meet resultaat is toegevoegd aan de flash opslag en het LoRaWAN bericht.
	
	BME280 toegevoegd, meet temperatuur, luchtvochtigheid en luchtdruk
		Meet resultaat is toegevoegd aan de flash opslag en het LoRaWAN bericht.
	
	Flash log payload is nu binair ipv hexadecimaal. Nu is de payload de helft kleiner.
	
	ERASE_MX_FLASH heeft nu een extra parameter waarmee twee opties kunnen worden geselecteerd; Erase fatfs log of Erase MX chip 
	
	BLE command toegevoegd:
		- BME280_CONVERSION_READ
		- BME280_CONVERSION_START
		- READ_AUDIO_ADC_CONFIG
		- WRITE_AUDIO_ADC_CONFIG
		- READ_AUDIO_ADC_CONVERSION
		- START_AUDIO_ADC_CONVERSION
		- ALARM_CONFIG_READ
		- ALARM_CONFIG_WRITE
		- ALARM_STATUS_READ
		
	Problemen met random resets in combinatie met de Nordic Power Profiler komen doordat de ingangspanning te hard daalt waneer er een LORaWAN bericht wordt verstuurd.
		Voor testen met de Nordic PKK is het aan te bevelen om de diode die aan de ingang zit te verwijderen of met een kabel te overbruggen.
		
	Firmware handleiding 1.3 staat nu op Google drive

2019_12_03 Release van Versie 1.2.2
	Fix voor het probleem met nieuwere smartphones die geen pin-code pop-up krijgen, omdat de encryptie mislukt.

2019_12_02 Release van Versie 1.2.1
	Recompile van release 1.2.0, maar dan zonder Pin code. Blijkt dat dit toch niet correct werkt op andere devices dan waar ik op heb getest.

2019_11_29 Release van Versie 1.2.0
	Flash logging toegevoegd voor meet data met als basis de Image Transfer voorbeeld: https://github.com/NordicPlayground/nrf52-ble-image-transfer-demo
		Voor de app ontwikkeling kan ook nog naar deze link worden gekeken: https://github.com/NordicPlayground/Android-Image-Transfer-Demo
		De snelheid is niet bepaald hoog met nRF Connect op een S5, maar dit komt grotedeels door de lage interval connectie en de MTU grote. Op mijn S5 wil het OS geen andere
		parameters instellen dan de instellingen bij het verbinden. De developer(https://devzone.nordicsemi.com/f/nordic-q-a/47503/bluetooth-5-0-file-transfer) van de OTS service zegt 
		snelheden te hebben behaald van 1200kbps, dus misschien dat er vanuit de App meer mogelijk is dan dat ik vanuit de peripheral kan behalen qua connectie interval.
		Ik heb wel de MTU standaard op 240 btes gezet ipv 23, omdat het onderhandelen met de client (S5 met nRF Connect) niks opleverde.
		
	BLE commands toegevoegd voor de Bootcount, FLASH_READ, FLASH_ERASE, FLASH_SIZE
	
	TX karakteristiek toegevoegd aan de BEEP service.
	
	Uitlezen van het flash log via de Beep control point en TX data karakteristiek
	
	Pincode bescherming aangezet. Pincode kan worden gereset door de reedswitch voor 30 seconden te bekrachtigen. De BEEPBASE geeft een pieptoon als deze is gereset.
		Dit betekend ook dat bonding nu verplicht is. Als een van de twee devices een vorige bond is vergeten kan dit een aantal pogingen kosten voordat de pincode pop-up komt. Als
		Encryptie mislukt op de BEEPBASE wordt de conenctie verbroken. Dit houdt in dat het verbinden een aantal keer herhaald moet worden voordat of de bestaande bond eerst gewist
		moet worden.
		Het bekrachtigen van de reedswitch wist alle bonden op de BEEPBASE.
	

2019_11_11 Release van Versie 1.1.1
	Release 1.1.1 gemaakt met de buzzer geactiveerd in firmware en een extra default tune op index 2. Beep protocol commando: 0x9102
	Programeer script aangepast zodat deze niet langer verwacht dat de hex file in een map genaamd "release staat."
	Firmware release 1.1.0 voor het gemak toegevoegd, aangezien die bijna gelijk is.
	Het probleem van de ontbrekende DS18B20 temperatuur in de LoRaWAN berichten was heel simpel: De decoder verwachte meer bytes dan er nog in het bericht aanwezig waren en stopte met decoderen.
	De temperatuur zat dus wel in het LoRaWAN bericht, maar in alle haast zag ik het DS18B20 READ commando niet in het hexadecimale bericht.

2019_11_04 Release van versie 1.1.0
	Eerste release van firmware.
	Bij verdere releases zal in dit document veranderingen worden bijgehouden.
	Beep Base Handleiding ge-update naar aanleiding van wijzigen van HX711 read and Write conversion command, HX711 state R/W command toegevoegd, Buzzer custom en default tune commands toegevoegd.
	
	
2019_11_01
	Vrijgave Beep Base handleiding 1V0