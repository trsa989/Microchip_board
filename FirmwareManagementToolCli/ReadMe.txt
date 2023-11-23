This is CLI version of firmware management tool.
For signing "other" images (aka g3-plc, DC, ...) use next arguments:
-s -i "fw\webmng_4.0-5776_armel.deb" -e ISKMO0Gw -f ISK-IMG-GWF-ISK -g 04

possible -e attributes:
ISK550EA, ISK550TA, ISK550EB, ISK550TB, ISK550EC, ISK550TC, ISK550ER, ISK550TR, 
ISK550EK, ISK550TK, ISKAMM550, ISKACM550, ISKMO3, ISKMO4, ISKMO5, ISKMO6HL6528, 
ISKMO6HL3450, ISKMO6HL8518, ISKMO6HL7692, ISKMO7wM-Bus, ISKMO7wM-BusA0, ISKMO0Gw,
ISKMO1Gw, ISKMO3Gw, ISKMO4Gw

where -g 04 represents ECDSA signature of image.

Choose signing certificate with option -c
-s -i "fw\ISK550TGENSEG_Application.hex" -c IeDevelopmentKey
valid certificate names: IeDevelopmentKey, IeInitialKey, IeProductionKey, IeSolentKey, IeKngKey, IeGenericKey

if you are using -n option to overload output file name, you need to add some extension (.bin, as in example: -n 'AM195_3.04_U_20191025.bin')

regarding option -i: now you can define path to folder, that includes hex files. Script will load all hex files (based on pattern *.hex)